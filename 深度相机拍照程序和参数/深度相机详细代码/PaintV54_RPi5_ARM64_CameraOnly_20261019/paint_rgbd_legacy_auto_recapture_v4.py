#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Orbbec SDK Python 1.1.4 legacy API | paperboard 2-point RGB-D V4.

Uses the pre-existing, successful orbbec_rgbd_auto_capture_v1 helpers
and orbbec_d2c_capture_with_provenance_v3 camera-parameter validation.
Never commands an AGV/robot, never certifies geometry or hand-eye calibration.
Never modifies previous collection files. See README for limitations.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
import time
import uuid
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

SCHEMA = 'paperboard_orbbec_legacy_v4_preliminary'
STATUS = 'PRELIMINARY_3D_NOT_CERTIFIED_NOT_FOR_ROBOT'
WH = (640, 480)
RGB_PROFILE = (640, 480, 30, 5)
DEPTH_PROFILE = (640, 400, 30, 11)


class RetryableFrameError(RuntimeError):
    pass


def fatal(msg):
    raise RuntimeError(msg)


def retry(msg):
    raise RetryableFrameError(msg)


def parse_uv(val):
    try:
        parts = val.split(',')
        if len(parts) != 2:
            raise ValueError('two components required')
        u, v = map(int, (part.strip() for part in parts))
    except (ValueError, AttributeError) as exc:
        raise argparse.ArgumentTypeError('点位格式必须为 u,v，例如 290,225') from exc
    if not (0 <= u < WH[0] and 0 <= v < WH[1]):
        raise argparse.ArgumentTypeError('点位超过640x480范围')
    return u, v


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as fh:
        for block in iter(lambda: fh.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def new_session(outroot):
    root = Path(outroot).expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    for _ in range(20):
        name = 'board_legacy_v4_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '_' + uuid.uuid4().hex[:8]
        location = root / name
        try:
            location.mkdir(exist_ok=False)
            return location
        except FileExistsError:
            continue
    fatal('无法创建唯一存档目录')


def write_json(file, obj):
    with Path(file).open('x', encoding='utf-8') as fh:
        json.dump(obj, fh, ensure_ascii=False, indent=2, allow_nan=False)
        fh.write('\n')


def write_png(file, bgr):
    ok, encoded = cv2.imencode('.png', bgr)
    if not ok or encoded is None:
        fatal('PNG编码失败: ' + str(file))
    with Path(file).open('xb') as fh:
        fh.write(encoded.tobytes())


def check_intrinsics(snapshot):
    if not isinstance(snapshot, dict):
        fatal('当前会话相机参数必须是字典快照')
    if snapshot.get('source') != 'SDK_getCameraParam_in_SAME_capture_session':
        fatal('不是本次会话的相机参数快照')
    k = snapshot.get('rgb_intrinsic')
    dk = snapshot.get('d2c_intrinsic')
    if not isinstance(k, dict) or not isinstance(dk, dict):
        fatal('缺少本次 SDK D2C/RGB 内参')
    for label, obj in [('rgb', k), ('d2c', dk)]:
        if (obj.get('width'), obj.get('height')) != WH:
            fatal(label + ' 内参尺寸不是640x480')
        for field in ('fx', 'fy', 'cx', 'cy'):
            value = obj.get(field)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                fatal(label + ' 内参无效: ' + field)
        if obj['fx'] <= 0 or obj['fy'] <= 0:
            fatal(label + ' 焦距无效')
        if not (-1 < obj['cx'] < 641 and -1 < obj['cy'] < 481):
            fatal(label + ' 主点异常')
    for field in ('fx', 'fy', 'cx', 'cy'):
        if abs(k[field] - dk[field]) > 1e-3:
            fatal('RGB与D2C内参不一致: ' + field)
    return {field: k[field] for field in ('fx','fy','cx','cy','width','height')}


def check_arrays(bgr, raw, scale):
    if not isinstance(bgr, np.ndarray) or bgr.shape != (480, 640, 3) or bgr.dtype != np.uint8:
        fatal('RGB BGR数组格式无效')
    if not isinstance(raw, np.ndarray) or raw.shape != (480, 640) or raw.dtype != np.uint16:
        fatal('SW-D2C数组必须为480x640 uint16')
    if isinstance(scale, bool) or not isinstance(scale, (int, float, np.floating)) or not math.isfinite(float(scale)) or not (0 < float(scale) < 100):
        fatal('Depth scale 非有限正数或范围异常')


def xyz_point(pid, uv, raw, scale, k):
    u, v = uv
    if u < 3 or v < 3 or u >= 637 or v >= 477:
        fatal(f'{pid} 不在允许完整7x7邻域的位置')
    center = int(raw[v, u])
    if not 0 < center < 65535:
        retry(f'{pid} 中心深度无效(raw={center})，重新采集')
    neighbor = raw[v-3:v+4, u-3:u+4]
    good = (neighbor > 0) & (neighbor < 65535)
    fraction = float(good.mean())
    if fraction < 0.9:
        retry(f'{pid} 7x7有效率 {fraction:.3f} < 0.9')
    values = neighbor[good].astype(np.float64) * float(scale)
    median = float(np.median(values))
    p10, p90 = [float(x) for x in np.percentile(values, [10, 90])]
    if p90 - p10 > 15.0:
        retry(f'{pid} 7x7深度跳变 {p90-p10:.2f}mm > 15mm')
    if abs(center * float(scale) - median) > 15.0:
        retry(f'{pid} 中心深度偏离邻域中位数过大')
    if not 100 <= median <= 2500:
        retry(f'{pid} 深度 {median:.2f}mm 超出纸板验证的100~2500mm区间')
    X = (u - float(k['cx'])) * median / float(k['fx'])
    Y = (v - float(k['cy'])) * median / float(k['fy'])
    return {
        'point_id': pid, 'pixel_uv': [u, v],
        'depth_raw_center': center, 'center_z_mm': center * float(scale),
        'depth_mm_used_median7x7': median,
        'local_valid_fraction': fraction,
        'local_p90_minus_p10_mm': p90-p10,
        'xyz_mm_pinhole_preliminary': [X, Y, median],
        'point_source': 'BOARD_FIXED_PIXELS_NOT_UNET',
    }


def try_two_points(raw, scale, k, p1, p2):
    if p1 == p2:
        fatal('P1和P2不允许相同')
    return [xyz_point('P1', p1, raw, scale, k),
            xyz_point('P2', p2, raw, scale, k)]


def require_sdk_modules():
    """No camera start. Imports real local 1.1.4 binding + legacy helpers."""
    try:
        import Pipeline
        import ObTypes
        from orbbec_rgbd_auto_capture_v1 import (decode_color, decode_depth,
            sdk_format_constants, select_profile, timestamp_fields)
        from orbbec_d2c_capture_with_provenance_v3 import (
            checked_camera_param, checked_timestamp_fields)
    except (ImportError, OSError) as exc:
        fatal('旧SDK/既有助手模块不可导入，禁止自动重装: ' + str(exc))
    if not hasattr(Pipeline, 'Pipeline') or not hasattr(Pipeline, 'Config'):
        fatal('旧SDK Pipeline/Config 缺失')
    if getattr(ObTypes, 'OB_PY_ALIGN_D2C_SW_MODE', None) is None:
        fatal('旧SDK SW-D2C 常量缺失')
    for name in ('OB_PY_SENSOR_COLOR', 'OB_PY_SENSOR_DEPTH', 'OB_PY_STREAM_VIDEO'):
        if getattr(ObTypes, name, None) is None:
            fatal('旧SDK 常量缺失: ' + name)
    return {'Pipeline': Pipeline, 'ObTypes': ObTypes,
            'decode_color': decode_color, 'decode_depth': decode_depth,
            'sdk_format_constants': sdk_format_constants,
            'select_profile': select_profile,
            'checked_camera_param': checked_camera_param,
            'checked_timestamp_fields': checked_timestamp_fields}


def capture(sdk, args, history):
    """One actual SDK Pipeline session; retries request fresh FrameSets.

    Structural problems do not automatically restart the device. All frame image
    buffers are copied BEFORE requesting another SDK frame.
    """
    Pipe = sdk['Pipeline']
    Ob = sdk['ObTypes']
    fmts = sdk['sdk_format_constants'](Ob)
    if fmts.get('OB_PY_FORMAT_MJPG') != 5:
        fatal('旧SDK MJPG 格式ID不符合既有成功profile')
    pipe = None
    started = False
    color_keeper = depth_keeper = None  # Retain profile wrappers for C++ lifetime.
    shutdown_warning = None
    accepted = None
    try:
        pipe = Pipe.Pipeline(None, None)
        color_keeper, color_prof, _ = sdk['select_profile'](
            pipe, Ob.OB_PY_SENSOR_COLOR, Ob.OB_PY_STREAM_VIDEO,
            RGB_PROFILE, index_hint=1)
        depth_keeper, depth_prof, _ = sdk['select_profile'](
            pipe, Ob.OB_PY_SENSOR_DEPTH, Ob.OB_PY_STREAM_VIDEO,
            DEPTH_PROFILE, index_hint=16)
        cfg = Pipe.Config()
        cfg.enableStream(color_prof)
        cfg.enableStream(depth_prof)
        cfg.setAlignMode(Ob.OB_PY_ALIGN_D2C_SW_MODE)
        pipe.start(cfg, None)
        started = True
        camera = sdk['checked_camera_param'](pipe.getCameraParam())
        k = check_intrinsics(camera)
        warmup_pairs = 0
        warmup_calls = 0
        while warmup_pairs < args.warmup and warmup_calls < args.warmup_max_waits:
            warmup_calls += 1
            try:
                fs = pipe.waitForFrames(args.wait_ms)
            except TimeoutError:
                continue
            if fs is None or fs.colorFrame() is None or fs.depthFrame() is None:
                continue
            warmup_pairs += 1
        if warmup_pairs < args.warmup:
            fatal('预热阶段未取得足够RGB+Depth帧；不开始正式测量')
        last_ts = None
        for attempt in range(1, args.max_capture_attempts + 1):
            entry = {'attempt_index':attempt, 'new_frameset_requested':True,
                     'robot_motion_authorized':False}
            try:
                # A None frameset is retryable. Other SDK exceptions remain structural
                # until real hardware logs justify classifying specific exceptions.
                try:
                    fs = pipe.waitForFrames(args.wait_ms)
                except TimeoutError as exc:
                    retry('waitForFrames等待超时，重新请求新帧: '+str(exc))
                if fs is None:
                    retry('waitForFrames返回空，重新请求新帧')
                cframe, dframe = fs.colorFrame(), fs.depthFrame()
                if cframe is None or dframe is None:
                    retry('Frameset缺少RGB或Depth，重新请求新帧')
                ct = sdk['checked_timestamp_fields'](cframe)
                dt = sdk['checked_timestamp_fields'](dframe)
                cst, dst = ct.get('systemTimeStamp'), dt.get('systemTimeStamp')
                if not isinstance(cst, int) or not isinstance(dst, int) or cst <= 0 or dst <= 0:
                    fatal('当前SDK帧缺少有效systemTimeStamp；无法证明重复取帧已前进')
                entry.update(rgb_system_timestamp_raw=cst,
                             depth_system_timestamp_raw=dst,
                             system_timestamp_delta_raw=abs(cst-dst),
                             timestamp_fields_clock_domain_verified=False)
                if last_ts is not None and (cst <= last_ts[0] or dst <= last_ts[1]):
                    retry('系统时间戳未前进，拒绝重复/旧帧')
                last_ts = (cst, dst)
                # This is ONLY a reported host timestamp diagnostic, not exposure sync.
                if abs(cst - dst) > args.max_system_timestamp_delta_raw:
                    retry('SDK系统时间戳原始差值超出诊断上限，重新采集')
                bgr, cfid, _ = sdk['decode_color'](cframe, fmts, width=640, height=480)
                if (int(dframe.width()), int(dframe.height())) != WH:
                    fatal('D2C输出网格不是640x480')
                raw, scale, dfid = sdk['decode_depth'](dframe, fmts, width=640, height=480)
                bgr = np.array(bgr, copy=True)
                raw = np.array(raw, copy=True)
                check_arrays(bgr, raw, scale)
                points = try_two_points(raw, scale, k, args.p1, args.p2)
                entry.update(result='ACCEPTED_PRELIMINARY_ONLY', rgb_format_id=int(cfid),
                             depth_format_id=int(dfid))
                history.append(entry)
                accepted = {
                    'bgr':bgr, 'raw':raw, 'scale':float(scale), 'k':k,
                    'points':points, 'camera_snapshot':camera,
                    'meta':{
                        'source':'SAME_LEGACY_SDK_FRAMESET_RGB_SW_D2C',
                        'accepted_attempt':attempt,
                        'rgb_system_timestamp_raw':cst,
                        'depth_system_timestamp_raw':dst,
                        'system_timestamp_delta_raw':abs(cst-dst),
                        'sdk_color_timestamp_fields_raw':ct,
                        'sdk_depth_timestamp_fields_raw':dt,
                        'timestamp_clock_domain_verified':False,
                        'exposure_synchronization_verified':False,
                        'geometric_alignment_verified':False,
                        'mirror_pixel_consistency_verified':False,
                        'robot_motion_authorized':False,
                    }
                }
                break
            except RetryableFrameError as exc:
                entry.update(result='REJECTED_RECAPTURE', reason=str(exc))
                history.append(entry)
                print('RECAPTURE {}/{}: {}'.format(attempt,args.max_capture_attempts,exc), flush=True)
                if attempt < args.max_capture_attempts and args.retry_delay_seconds:
                    time.sleep(args.retry_delay_seconds)
            except Exception as exc:
                entry.update(result='FATAL_NOT_RETRYABLE',reason=f'{type(exc).__name__}: {exc}')
                history.append(entry)
                raise
        if accepted is None:
            fatal(f'最多{args.max_capture_attempts}次新Frameset均不合格；请检查点位、目标、反光或设备')
    finally:
        if started:
            try:
                pipe.stop()
            except Exception as exc:
                shutdown_warning = f'{type(exc).__name__}: {exc}'
                print('SDK_STOP_WARNING:', shutdown_warning, file=sys.stderr, flush=True)
    if shutdown_warning:
        accepted['meta']['stream_stop_warning'] = shutdown_warning
        accepted['meta']['stream_stop_clean'] = False
    else:
        accepted['meta']['stream_stop_clean'] = True
    return accepted


def save_accepted(session, accepted, args, history):
    bgr, raw, scale, k = (accepted[field] for field in ('bgr', 'raw', 'scale', 'k'))
    points = try_two_points(raw, scale, k, args.p1, args.p2)
    p1 = np.asarray(points[0]['xyz_mm_pinhole_preliminary'], dtype=np.float64)
    p2 = np.asarray(points[1]['xyz_mm_pinhole_preliminary'], dtype=np.float64)
    distance = float(np.linalg.norm(p1-p2))
    rgb_path = session/'rgb_001.png'
    depth_path = session/'d2c_depth_raw.npy'
    overlay_path = session/'rgb_two_points_preliminary.png'
    camera_path = session/'camera_parameters_same_session.json'
    write_png(rgb_path, bgr)
    with depth_path.open('xb') as fh:
        np.save(fh, raw, allow_pickle=False)
    overlay = bgr.copy()
    for pt in points:
        u, v = pt['pixel_uv']
        cv2.circle(overlay, (u,v), 8, (0,255,0), 2)
        cv2.circle(overlay, (u,v), 2, (0,255,0), -1)
        cv2.putText(overlay,
                    f"{pt['point_id']} Z={pt['depth_mm_used_median7x7']:.1f}mm",
                    (min(u+10,500), max(v-10,15)), cv2.FONT_HERSHEY_SIMPLEX,
                    .5, (0,0,255), 1, cv2.LINE_AA)
    write_png(overlay_path, overlay)
    write_json(camera_path, accepted['camera_snapshot'])
    # Check copies on disk before saving a success report.
    check_png = cv2.imread(str(rgb_path), cv2.IMREAD_COLOR)
    if check_png is None or not np.array_equal(check_png,bgr):
        fatal('已保存RGB图片回读校验失败')
    if not np.array_equal(np.load(str(depth_path), allow_pickle=False),raw):
        fatal('已保存Depth数组回读校验失败')
    digests = {name:sha(session/name) for name in
               ('rgb_001.png','d2c_depth_raw.npy','rgb_two_points_preliminary.png',
                'camera_parameters_same_session.json')}
    report = {
        'schema':SCHEMA, 'status':STATUS,
        'robot_motion_authorized':False, 'mode':'board',
        'point_source':'BOARD_FIXED_PIXELS_NOT_UNET',
        'xyz_coordinate_frame':'APPROX_D2C_PROCESSED_PINHOLE_FRAME_NOT_ROBOT',
        'camera_geometry_alignment_validated':False,
        'camera_distortion_model_verified':False,
        'mirroring_verified_against_actual_pixels':False,
        'human_confirmed_board_points':False,
        'intrinsic_used':k, 'depth_scale_mm_per_raw':scale,
        'rgb_depth_pairing':accepted['meta'],
        'points':points, 'distance_between_points_mm_preliminary':distance,
        'source_files_sha256':digests,
        'warnings':[
            'Fixed board pixels; verify markers on real physical board manually',
            'Same SDK FrameSet is not certified acquisition-time synchronization',
            'SW-D2C 640x480 grid alone does not verify geometric registration',
            'Mirror/distortion/absolute depth accuracy/hand-eye/probe normal unverified',
            'Camera XYZ is preliminary and MUST NOT be commanded to robot',
        ]
    }
    write_json(session/'capture_manifest.json',{
        'schema':SCHEMA,'status':STATUS,'rgb_file':'rgb_001.png',
        'depth_file':'d2c_depth_raw.npy', 'source_files_sha256':digests,
        'intrinsic_used':k, 'depth_scale_mm_per_raw':scale,
        'rgb_depth_pairing':accepted['meta'],
        'robot_motion_authorized':False,
    })
    write_json(session/'retry_diagnostics.json',{
        'status':'PRELIMINARY_SUCCESS_NOT_FOR_ROBOT',
        'attempts':history, 'capture_attempts':len(history),
        'robot_motion_authorized':False,
    })
    # LAST: a report exists only after RGB, depth, parameters, manifest, and
    # diagnostics have already been stored. A partial session is not a success.
    write_json(session/'two_points_xyz_PRELIMINARY_NOT_FOR_ROBOT.json', report)
    return report


def build_parser():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--preflight', action='store_true', help='仅检查旧SDK/助手模块导入，不启动相机')
    ap.add_argument('--out-root', type=Path, default=Path('runs/board_rgbd_legacy_v4'))
    ap.add_argument('--p1', type=parse_uv, default=(290,225))
    ap.add_argument('--p2', type=parse_uv, default=(350,255))
    ap.add_argument('--max-capture-attempts',type=int,default=12)
    ap.add_argument('--retry-delay-seconds',type=float,default=0.25)
    ap.add_argument('--wait-ms',type=int,default=750)
    ap.add_argument('--warmup',type=int,default=5)
    ap.add_argument('--warmup-max-waits',type=int,default=30)
    ap.add_argument('--max-system-timestamp-delta-raw',type=int,default=100,
                    help='仅诊断阈值，原字段值未经时钟/曝光校准')
    return ap


def main(argv=None):
    args = build_parser().parse_args(argv)
    if (args.p1 == args.p2 or not (1 <= args.max_capture_attempts <= 120)
            or not (0 <= args.retry_delay_seconds <= 5)
            or not (100 <= args.wait_ms <= 2000)
            or not (0 <= args.warmup <= 30)
            or not (args.warmup <= args.warmup_max_waits <= 300)
            or not (1 <= args.max_system_timestamp_delta_raw <= 1000)):
        print('ARGUMENTS_INVALID: 请核对点位、等待时间及重拍上限',file=sys.stderr)
        return 2
    history = []
    session = None
    try:
        sdk = require_sdk_modules()
        if args.preflight:
            print('LEGACY_V4_PREFLIGHT_PASS_NO_CAMERA_START')
            print('PYTHON:', sys.executable)
            return 0
        session = new_session(args.out_root)
        accepted = capture(sdk,args,history)
        report = save_accepted(session,accepted,args,history)
        print('BOARD_LEGACY_V4_SUCCESS_PRELIMINARY_ONLY')
        print('OUT:',session)
        print('CAPTURE_ATTEMPTS:',len(history))
        for pt in report['points']:
            print(pt['point_id'],'uv=',tuple(pt['pixel_uv']),
                  'Z=',pt['depth_mm_used_median7x7'],
                  'XYZ_mm=',[round(x,3) for x in pt['xyz_mm_pinhole_preliminary']])
        print('PAIR_DISTANCE_PRELIMINARY_MM=',round(report['distance_between_points_mm_preliminary'],3))
        print('ROBOT_MOTION_AUTHORIZED: FALSE')
        return 0
    except Exception as exc:
        print('SAFE_STOP_NO_ROBOT:',type(exc).__name__,str(exc),file=sys.stderr,flush=True)
        if session is not None:
            try:
                write_json(session/'retry_diagnostics.json',{
                    'status':'FAILED_NO_ROBOT','attempts':history,
                    'robot_motion_authorized':False})
            except (OSError, ValueError) as why:
                print('DIAGNOSTIC_WRITE_WARNING:',why,file=sys.stderr)
            try:
                write_json(session/'ERROR_STOP.json',{
                    'status':'FAILED_NO_ROBOT','exception_type':type(exc).__name__,
                    'error':str(exc),'robot_motion_authorized':False})
            except (OSError, ValueError) as why:
                print('ERROR_RECORD_WARNING:',why,file=sys.stderr)
            print('OUT:',session,file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
