#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""One fresh real RGB+SW-D2C FrameSet using the VERIFIED legacy Orbbec API.
Runs ONLY under orbbec_py39; imports the unmodified V4 validation/helpers.
No robot calls. Writes an exclusive per-attempt artifact after stopping camera.
"""
import argparse
import hashlib
import json
import sys
from pathlib import Path

import cv2
import numpy as np

from paint_rgbd_legacy_auto_recapture_v4 import (check_arrays, check_intrinsics,
    require_sdk_modules, RGB_PROFILE, DEPTH_PROFILE)

class RecoverableCapture(Exception):
    pass

def sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as fh:
        for block in iter(lambda: fh.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()

def store_json(path, value):
    with path.open('x', encoding='utf-8') as fh:
        json.dump(value, fh, ensure_ascii=False, indent=2, allow_nan=False)
        fh.write('\n')

def acquire(sdk, warmup, max_waits, wait_ms, timestamp_delta_limit):
    Pipe, Ob = sdk['Pipeline'], sdk['ObTypes']
    fmts = sdk['sdk_format_constants'](Ob)
    if fmts.get('OB_PY_FORMAT_MJPG') != 5:
        raise RuntimeError('Unexpected MJPG format identifier')
    pipe, started = None, False
    color_keeper = depth_keeper = None
    result = None
    mjpeg_decode_failures = 0
    try:
        pipe = Pipe.Pipeline(None, None)
        color_keeper, cp, _ = sdk['select_profile'](
            pipe, Ob.OB_PY_SENSOR_COLOR, Ob.OB_PY_STREAM_VIDEO,
            RGB_PROFILE, index_hint=1)
        depth_keeper, dp, _ = sdk['select_profile'](
            pipe, Ob.OB_PY_SENSOR_DEPTH, Ob.OB_PY_STREAM_VIDEO,
            DEPTH_PROFILE, index_hint=16)
        cfg = Pipe.Config()
        cfg.enableStream(cp)
        cfg.enableStream(dp)
        cfg.setAlignMode(Ob.OB_PY_ALIGN_D2C_SW_MODE)
        pipe.start(cfg, None)
        started = True
        snapshot = sdk['checked_camera_param'](pipe.getCameraParam())
        k = check_intrinsics(snapshot)
        good_warmup = 0
        prev = None
        for _ in range(max_waits):
            try:
                fs = pipe.waitForFrames(wait_ms)
            except TimeoutError:
                continue
            if fs is None:
                continue
            cf, df = fs.colorFrame(), fs.depthFrame()
            if cf is None or df is None:
                continue
            ct = sdk['checked_timestamp_fields'](cf)
            dt = sdk['checked_timestamp_fields'](df)
            cst, dst = ct.get('systemTimeStamp'), dt.get('systemTimeStamp')
            if (not isinstance(cst, int) or not isinstance(dst, int)
                    or cst <= 0 or dst <= 0):
                raise RuntimeError('SDK timestamps absent or invalid: cannot verify fresh pair')
            if prev is not None and (cst <= prev[0] or dst <= prev[1]):
                raise RecoverableCapture('Duplicate/older RGB or depth system timestamp')
            prev = (cst, dst)
            if good_warmup < warmup:
                good_warmup += 1
                continue
            if abs(cst-dst) > timestamp_delta_limit:
                raise RecoverableCapture('Timestamp delta exceeds DIAGNOSTIC bound')
            try:
                bgr, colorfmt, _ = sdk['decode_color'](cf, fmts, width=640, height=480)
            except RuntimeError as exc:
                # Observed in the user's live SDK log: an occasional MJPEG
                # frame is undecodable. Only THIS exact signature is handled
                # as a bad frame, never as a camera/configuration error.
                if str(exc) != 'COLOR_MJPEG_DECODE_FAILED':
                    raise
                mjpeg_decode_failures += 1
                print('V5_4_DROP_BAD_MJPEG_FRAME: count={} rgb_ts={} depth_ts={} (requesting new Frameset)'.format(
                    mjpeg_decode_failures, cst, dst), file=sys.stderr, flush=True)
                continue
            if (int(df.width()),int(df.height())) != (640,480):
                raise RuntimeError('SW-D2C output is not 640x480')
            raw, scale, depthfmt = sdk['decode_depth'](df, fmts, width=640, height=480)
            bgr, raw = np.array(bgr, copy=True), np.array(raw, copy=True)
            check_arrays(bgr,raw,scale)
            if not np.any((raw > 0) & (raw < 65535)):
                raise RecoverableCapture('Depth map has zero valid pixels')
            if mjpeg_decode_failures:
                print('V5_4_MJPEG_RECOVERED: bad_frames={} accepted_rgb_ts={} accepted_depth_ts={}'.format(
                    mjpeg_decode_failures, cst, dst), flush=True)
            result = (bgr, raw, float(scale), snapshot, k,
                {'color_format_id':int(colorfmt), 'depth_format_id':int(depthfmt),
                 'rgb_system_timestamp_raw':cst,
                 'depth_system_timestamp_raw':dst,
                 'system_timestamp_delta_raw':abs(cst-dst),
                 'sdk_color_timestamp_fields_raw':ct,
                 'sdk_depth_timestamp_fields_raw':dt,
                 'source':'ONE_SDK_FRAMESET_LEGACY_SW_D2C',
                 'exposure_sync_verified':False,
                 'rgbd_geometric_registration_verified':False,
                 'robot_motion_authorized':False})
            break
        if result is None:
            raise RecoverableCapture('No fresh complete RGB+SW-D2C pair after bounded waits; '
                                     'bad_mjpeg_frames={}'.format(mjpeg_decode_failures))
    finally:
        if started:
            pipe.stop()  # stop failures are fatal; do not silently accept data
    return result

def save(out, value):
    bgr,raw,scale,params,k,meta = value
    if out.exists() or out.is_symlink() or not out.parent.is_dir():
        raise FileExistsError('Camera attempt output exists or parent missing: '+str(out))
    out.mkdir(exist_ok=False)
    success = False
    try:
        ret, png = cv2.imencode('.png', bgr)
        if not ret or png is None:
            raise RuntimeError('RGB PNG encoding failed')
        with (out/'rgb_001.png').open('xb') as fh:
            fh.write(png.tobytes())
        with (out/'d2c_depth_raw.npy').open('xb') as fh:
            np.save(fh, raw, allow_pickle=False)
        store_json(out/'camera_parameters_same_session.json',params)
        content = ['rgb_001.png','d2c_depth_raw.npy','camera_parameters_same_session.json']
        manifest = {'schema':'paint_rgbd_v5_legacy_capture_v1',
            'rgb_file':content[0], 'depth_file':content[1],
            'camera_params_file':content[2], 'depth_scale_mm_per_raw':scale,
            'intrinsic_used':k, 'rgb_depth_pairing':meta,
            'source_files_sha256':{name:sha256(out/name) for name in content},
            'robot_motion_authorized':False}
        if not np.array_equal(cv2.imread(str(out/content[0]), cv2.IMREAD_COLOR), bgr):
            raise RuntimeError('RGB disk round-trip mismatch')
        if not np.array_equal(np.load(str(out/content[1]),allow_pickle=False),raw):
            raise RuntimeError('Depth disk round-trip mismatch')
        store_json(out/'capture_manifest.json',manifest)
        store_json(out/'CAPTURE_COMPLETE.json',{'sha256_capture_manifest':sha256(out/'capture_manifest.json'),
            'robot_motion_authorized':False})
        success = True
    finally:
        if not success:
            print('CAMERA_OUTPUT_INCOMPLETE:',out,file=sys.stderr,flush=True)

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--preflight',action='store_true')
    p.add_argument('--out',type=Path)
    p.add_argument('--warmup',type=int,default=5)
    p.add_argument('--max-waits',type=int,default=30)
    p.add_argument('--wait-ms',type=int,default=750)
    p.add_argument('--timestamp-delta-diagnostic-limit',type=int,default=100)
    a=p.parse_args()
    if (not 0<=a.warmup<=30 or not a.warmup+1<=a.max_waits<=120
        or not 100<=a.wait_ms<=2000
        or not 1<=a.timestamp_delta_diagnostic_limit<=1000):
        print('CAMERA_ARGUMENTS_INVALID',file=sys.stderr);return 12
    try:
        sdk=require_sdk_modules()
        if a.preflight:
            print('V5_LEGACY_CAMERA_IMPORT_PASS_NO_CAMERA_START',flush=True)
            return 0
        if a.out is None:
            print('Camera output --out is required',file=sys.stderr);return 12
        value=acquire(sdk,a.warmup,a.max_waits,a.wait_ms,a.timestamp_delta_diagnostic_limit)
        save(a.out,value)
        print('V5_4_CAMERA_ONE_FRESH_FRAMESET_PASS:',a.out,flush=True)
        return 0
    except RecoverableCapture as exc:
        print('V5_4_CAMERA_RECAPTURE:',str(exc),file=sys.stderr,flush=True)
        return 11
    except Exception as exc:
        print('V5_4_CAMERA_FATAL:',type(exc).__name__,str(exc),file=sys.stderr,flush=True)
        return 12

if __name__=='__main__':
    sys.exit(main())
