#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Orbbec 1.1.4 SW-D2C capture V3: reject empty depth and save provenance.

This preserves the SDK calls and profile configuration of the previously
successful orbbec_d2c_sw_validation_v1.py. It NEVER controls a robot.
All output is placed in a NEW directory. Capture is diagnostic, not calibration.
"""
import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np

from orbbec_rgbd_auto_capture_v1 import (
    decode_color, decode_depth, sdk_format_constants,
    select_profile, timestamp_fields, require,
)

COLOR_PROFILE = [640, 480, 30, 5]
DEPTH_PROFILE = [640, 400, 30, 11]
D2C_GRID = [640, 480]


def sha256(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def checked_intrinsic(obj, name):
    require(isinstance(obj, dict), name + '_NOT_DICT')
    require(obj.get('width') == 640 and obj.get('height') == 480,
            name + '_NOT_640x480')
    for key in ('fx', 'fy', 'cx', 'cy'):
        value = obj.get(key)
        require(type(value) in (int, float) and math.isfinite(value),
                name + '_INVALID_' + key)
        if key in ('fx', 'fy'):
            require(value > 0, name + '_NONPOSITIVE_' + key)
    return dict(obj)


def checked_camera_param(cam):
    """Only documented, *observed* dictionary members; do not infer semantics."""
    require(isinstance(cam, dict), 'SDK_CAMERA_PARAM_NOT_DICT')
    keys = ('depthIntrinsic', 'rgbIntrinsic', 'depthDistortion',
            'rgbDistortion', 'transform', 'isMirrored')
    require(all(key in cam for key in keys), 'SDK_CAMERA_PARAM_KEYS_MISSING')
    depth_intrinsic = checked_intrinsic(cam['depthIntrinsic'], 'D2C_INTRINSIC')
    rgb_intrinsic = checked_intrinsic(cam['rgbIntrinsic'], 'RGB_INTRINSIC')
    for key in ('depthDistortion', 'rgbDistortion'):
        vals = cam[key]
        require(isinstance(vals, dict) and vals, 'INVALID_' + key)
        for val in vals.values():
            require(type(val) in (int, float) and math.isfinite(val),
                    'NONFINITE_' + key)
    require(type(cam['isMirrored']) is bool, 'SDK_MIRROR_FLAG_NOT_BOOLEAN')
    # SDK transform direction/translation units must remain unasserted here.
    require(isinstance(cam['transform'], dict), 'SDK_TRANSFORM_NOT_DICT')
    # Also force strict JSON serializability, rejecting NaN and unknown objects.
    json.dumps(cam, allow_nan=False)
    return {
        'schema': 'orbbec_d2c_parameter_snapshot_v1',
        'source': 'SDK_getCameraParam_in_SAME_capture_session',
        'device_serial': None,
        'device_serial_evidence': 'NOT_AVAILABLE_IN_THIS_VERIFIED_PYTHON_API_PATH',
        'firmware_log': None,
        'align_mode': 'OB_PY_ALIGN_D2C_SW_MODE',
        'image_size_wh': D2C_GRID,
        'rgb_intrinsic': rgb_intrinsic,
        'd2c_intrinsic': depth_intrinsic,
        'rgb_distortion': dict(cam['rgbDistortion']),
        'd2c_distortion': dict(cam['depthDistortion']),
        'sdk_isMirrored': cam['isMirrored'],
        'sdk_raw_transform_direction_and_units': 'UNVERIFIED',
        'sdk_raw_camera_param': cam,
        'actual_depth_mirror_consistency_verified': False,
        'actual_rgb_mirror_consistency_verified': False,
        'rgb_depth_timestamp_sync_verified': False,
        'rgbd_geometric_registration_accuracy_verified': False,
        'hand_eye_verified': False,
        'robot_motion_authorized': False,
    }


def checked_timestamp_fields(fr):
    """Retain original SDK values without guessing their common time base."""
    out = timestamp_fields(fr)
    require(isinstance(out, dict), 'TIMESTAMP_HELPER_RESULT_NOT_DICT')
    for key in ('timeStampUs', 'timeStamp', 'systemTimeStamp'):
        if key in out:
            require(type(out[key]) is int and out[key] >= 0,
                    'TIMESTAMP_FIELD_INVALID_' + key)
    return out


def diagnostic_images(bgr, raw, scale):
    valid = (raw > 0) & (raw < 65535)
    d_mm = raw.astype(np.float32) * np.float32(scale)
    preview = np.zeros_like(bgr)
    if np.any(valid):
        lo, hi = np.percentile(d_mm[valid], (5, 95))
        if hi <= lo:
            hi = lo + 1
        norm = np.clip((d_mm-lo)/(hi-lo), 0, 1)
        preview = cv2.applyColorMap(np.round(norm*255).astype(np.uint8), cv2.COLORMAP_TURBO)
        preview[~valid] = 0
    overlay = bgr.copy()
    # cv2.addWeighted(empty_array, ...) returns None in OpenCV: do not reshape it.
    if np.any(valid):
        overlay[valid] = cv2.addWeighted(
            bgr[valid].reshape(-1, 1, 3), .65,
            np.full((int(valid.sum()), 1, 3), (50, 190, 50), np.uint8),
            .35, 0).reshape(-1, 3)
    return preview, overlay


def run(args):
    out = Path(args.out).expanduser().resolve()
    require(out.parent.is_dir(), 'OUTPUT_PARENT_NOT_DIRECTORY')
    require(not out.exists() and not out.is_symlink(), 'OUTPUT_ALREADY_EXISTS')
    import Pipeline
    import ObTypes
    mode = getattr(ObTypes, 'OB_PY_ALIGN_D2C_SW_MODE', None)
    require(mode is not None, 'SDK_SW_D2C_NOT_AVAILABLE')
    fmts = sdk_format_constants(ObTypes)
    require(fmts.get('OB_PY_FORMAT_MJPG') == 5, 'SDK_MJPG_FORMAT_NOT_EXPECTED')
    pipe = None
    started = False
    color_profile_keeper = depth_profile_keeper = None
    cam_evidence = None
    captures = []
    try:
        pipe = Pipeline.Pipeline(None, None)
        color_profile_keeper, cp, _ = select_profile(
            pipe, ObTypes.OB_PY_SENSOR_COLOR, ObTypes.OB_PY_STREAM_VIDEO,
            tuple(COLOR_PROFILE), index_hint=1)
        depth_profile_keeper, dp, _ = select_profile(
            pipe, ObTypes.OB_PY_SENSOR_DEPTH, ObTypes.OB_PY_STREAM_VIDEO,
            tuple(DEPTH_PROFILE), index_hint=16)
        cfg = Pipeline.Config()
        cfg.enableStream(cp)
        cfg.enableStream(dp)
        cfg.setAlignMode(mode)
        pipe.start(cfg, None)
        started = True
        # Unlike V1, this result MUST be read in the actual capture session.
        cam_evidence = checked_camera_param(pipe.getCameraParam())
        print('SAME_SESSION_CAMERA_PARAMETERS_READ', flush=True)
        seen = 0
        for _ in range(args.max_waits):
            fs = pipe.waitForFrames(args.wait_ms)
            if fs is None:
                continue
            cf, df = fs.colorFrame(), fs.depthFrame()
            if cf is None or df is None:
                continue
            seen += 1
            if seen <= args.warmup:
                continue
            bgr, cfid, _ = decode_color(cf, fmts, width=640, height=480)
            dw, dh = int(df.width()), int(df.height())
            require((dw, dh) == (640, 480), 'D2C_GRID_NOT_RGB_640x480')
            raw, scale, dfid = decode_depth(df, fmts, width=640, height=480)
            require(isinstance(bgr, np.ndarray) and bgr.shape == (480,640,3) and bgr.dtype == np.uint8,
                    'RGB_ARRAY_INVALID')
            require(isinstance(raw, np.ndarray) and raw.shape == (480,640) and raw.dtype == np.uint16,
                    'D2C_ARRAY_INVALID')
            require(type(scale) in (int,float,np.float32,np.float64) and math.isfinite(scale) and scale > 0,
                    'DEPTH_SCALE_INVALID')
            cts = checked_timestamp_fields(cf)
            dts = checked_timestamp_fields(df)
            cst, dst = cts.get('systemTimeStamp'), dts.get('systemTimeStamp')
            delta = (cst - dst) if cst is not None and dst is not None else None
            valid = (raw > 0) & (raw < 65535)
            i = len(captures) + 1
            captures.append({
                'rgb_bgr': np.array(bgr, copy=True),
                'depth_raw': np.array(raw, copy=True),
                'scale': float(scale),
                'meta': {
                    'frame_index': i,
                    'color_frame_format': int(cfid),
                    'depth_frame_format': int(dfid),
                    'color_system_timestamp_ms': cst,
                    'depth_system_timestamp_ms': dst,
                    'timestamp_difference_ms_diagnostic_only': delta,
                    'color_sdk_timestamp_fields_raw': cts,
                    'depth_sdk_timestamp_fields_raw': dts,
                    'depth_valid_fraction': float(valid.mean()),
                    'central_20pct_valid_fraction': float(valid[192:288,256:384].mean()),
                }
            })
            center_valid = int(valid[192:288, 256:384].sum())
            print('PAIR %02d/%02d valid_px=%d/%d center20_valid_px=%d/%d '
                  'system_delta_raw=%s' %
                  (i, args.frames, int(valid.sum()), int(valid.size),
                   center_valid, 96*128, delta), flush=True)
            if i == args.frames:
                break
    finally:
        if started:
            pipe.stop()
            print('STREAM_STOPPED', flush=True)
    require(len(captures) == args.frames, 'INCOMPLETE_CAPTURE')
    # A recorded pair is not necessarily a measurement: it may contain all-zero depth.
    # Reject it before creating any output directory. Do not invent replacement depth.
    invalid_frames = [p['meta']['frame_index'] for p in captures
                      if p['meta']['depth_valid_fraction'] == 0.0]
    require(not invalid_frames,
            'EMPTY_DEPTH_FRAMES: %s; no 3D possible. Verify actual scene, depth '
            'range, IR illumination and SDK frame contents; old data unchanged.'
            % invalid_frames)
    # Prepare a new directory only after successful capture/stop and data screening.
    require(not out.exists(), 'OUTPUT_ALREADY_EXISTS_BEFORE_WRITE')
    out.mkdir(mode=0o755, exist_ok=False)
    hashes = []
    for i, cap in enumerate(captures, start=1):
        bgr, raw, scale = cap['rgb_bgr'], cap['depth_raw'], cap['scale']
        preview, overlay = diagnostic_images(bgr, raw, scale)
        for name, image in (
            ('rgb_%03d.png'%i, bgr),
            ('depth_preview_%03d_NOT_METRIC.png'%i, preview),
            ('rgb_depth_valid_overlay_%03d_NOT_CALIBRATED.png'%i, overlay),
        ):
            require(cv2.imwrite(str(out/name), image), 'PNG_SAVE_FAILED: ' + name)
        hashes.append({'rgb_png_sha256': sha256(out/('rgb_%03d.png'%i))})
    np.save(out/'depth_d2c_sw_raw_uint16.npy', np.stack([p['depth_raw'] for p in captures],0), allow_pickle=False)
    np.save(out/'depth_scale_mm_per_raw.npy', np.asarray([p['scale'] for p in captures],np.float64), allow_pickle=False)
    (out/'camera_parameters_same_session.json').write_text(
        json.dumps(cam_evidence, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    summary = {
        'source_file_hashes_sha256': {
            'camera_parameters_same_session.json': sha256(out/'camera_parameters_same_session.json'),
            'depth_d2c_sw_raw_uint16.npy': sha256(out/'depth_d2c_sw_raw_uint16.npy'),
            'depth_scale_mm_per_raw.npy': sha256(out/'depth_scale_mm_per_raw.npy'),
        },
        'diagnostic': 'SW_D2C_CAPTURE_V3_EMPTY_DEPTH_GATED_SAME_SESSION_NOT_CALIBRATION_APPROVAL',
        'requested_color': COLOR_PROFILE, 'requested_depth': DEPTH_PROFILE,
        'result_depth_grid': D2C_GRID,
        'requested_align_mode': 'OB_PY_ALIGN_D2C_SW_MODE',
        'frame_count': len(captures),
        'camera_parameters_file': 'camera_parameters_same_session.json',
        'same_session_camera_params_read': True,
        'camera_serial_number': None,
        'camera_serial_verified': False,
        'time_sync_validated': False,
        'geometry_alignment_validated': False,
        'calibration_accuracy_validated': False,
        'mirror_pixel_consistency_verified': False,
        'robot_motion_authorized': False,
        'frames': [{**p['meta'], **hashes[i]} for i,p in enumerate(captures)],
    }
    (out/'d2c_probe_summary.json').write_text(
        json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    (out/'D2C_GRID_DIAGNOSTIC_COMPLETE.txt').write_text(
        'D2C_OUTPUT_GRID_CAPTURED_NOT_GEOMETRY_OR_ROBOT_APPROVAL\n', encoding='ascii')
    print('SAME_SESSION_PROVENANCE_CAPTURE_COMPLETE:', out, flush=True)
    print('SERIAL_MIRROR_GEOMETRY_EXPOSURE_SYNC: NOT_VERIFIED', flush=True)
    print('ROBOT_MOTION_AUTHORIZED: FALSE', flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out', required=True)
    ap.add_argument('--frames', type=int, default=3)
    ap.add_argument('--warmup', type=int, default=5)
    ap.add_argument('--wait-ms', type=int, default=750)
    ap.add_argument('--max-waits', type=int, default=60)
    a = ap.parse_args()
    if not (1 <= a.frames <= 6 and 0 <= a.warmup <= 30 and
            a.frames + a.warmup <= a.max_waits <= 300 and
            100 <= a.wait_ms <= 2000):
        ap.error('INVALID_CAPTURE_ARGUMENTS')
    try:
        run(a)
    except Exception as exc:
        print('STOP:', type(exc).__name__, str(exc), file=sys.stderr, flush=True)
        raise SystemExit(2)


if __name__ == '__main__':
    main()
