#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""One-shot Orbbec RGB-only or RGB+depth capture, legacy SDK 1.1.4 / Python 3.9.

No GUI, no manual keys, no robot commands, no calibration assumptions.
For diagnosis and recording ONLY; stored RGB and depth ARE NOT registered,
clock-synchronized, or authorized for robot motion.
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np


def require(cond, message):
    if not cond:
        raise RuntimeError(message)


def sdk_format_constants(obtypes):
    keys = (
        'OB_PY_FORMAT_MJPG', 'OB_PY_FORMAT_RGB888', 'OB_PY_FORMAT_BGR',
        'OB_PY_FORMAT_YUYV', 'OB_PY_FORMAT_YUY2', 'OB_PY_FORMAT_UYVY',
        'OB_PY_FORMAT_Y16', 'OB_PY_FORMAT_Y11', 'OB_PY_FORMAT_Y12',
        'OB_PY_FORMAT_MJPG_TO_BGR888', 'OB_PY_FORMAT_MJPG_TO_RGB888',
    )
    return {key: int(getattr(obtypes, key)) for key in keys if hasattr(obtypes, key)}


def select_profile(pipe, sensor, video_type, expectation, *, index_hint=None):
    profiles = pipe.getStreamProfileList(sensor)
    count = int(profiles.count())
    require(1 <= count <= 256, f'PROFILE_COUNT_INVALID: {count}')
    matches = []
    for i in range(count):
        profile = profiles.getProfile(i).toConcreteStreamProfile(video_type)
        vals = (int(profile.width()), int(profile.height()),
                int(profile.fps()), int(profile.format()))
        if vals == expectation:
            matches.append((i, profile))
    print('PROFILE_AVAILABLE:', count, 'EXPECT:', expectation,
          'MATCH_INDICES:', [p[0] for p in matches], flush=True)
    require(len(matches) > 0, f'REQUIRED_PROFILE_NOT_FOUND: {expectation}')
    if index_hint is not None:
        matches = [p for p in matches if p[0] == index_hint]
        require(len(matches) == 1, f'EXPECTED_PROFILE_INDEX_MISMATCH: {index_hint}')
    i, profile = matches[0]
    print('SELECTED_PROFILE:', i, expectation, flush=True)
    return profiles, profile, i


def safe_frame_bytes(frame):
    declared = int(frame.dataSize())
    require(0 < declared <= 60 * 1024 * 1024, f'FRAME_SIZE_UNSAFE: {declared}')
    data = frame.data()
    require(not isinstance(data, int), 'FRAME_DATA_POINTER_NOT_BUFFER')
    try:
        mv = memoryview(data)
    except TypeError as exc:
        raise RuntimeError(f'FRAME_DATA_NO_BUFFER: {exc}') from exc
    require(mv.nbytes == declared,
            f'FRAME_BYTE_COUNT_MISMATCH: {mv.nbytes} vs {declared}')
    return mv.tobytes()


def decode_color(frame, constants, width=640, height=480):
    require((int(frame.width()), int(frame.height())) == (width, height),
            f'COLOR_DIMENSION_MISMATCH: {frame.width()}x{frame.height()}')
    fmt = int(frame.format())
    body = safe_frame_bytes(frame)
    def isfmt(name):
        return fmt == constants.get(name, -999999)

    if isfmt('OB_PY_FORMAT_MJPG'):
        # Compressed JPEG size varies. Do not reshape it to fixed RGB bytes.
        img = cv2.imdecode(np.frombuffer(body, dtype=np.uint8), cv2.IMREAD_COLOR)
        require(img is not None, 'COLOR_MJPEG_DECODE_FAILED')
    elif isfmt('OB_PY_FORMAT_RGB888'):
        require(len(body) == width * height * 3, 'COLOR_RGB888_LENGTH_MISMATCH')
        rgb = np.frombuffer(body, dtype=np.uint8).reshape(height, width, 3)
        img = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    elif isfmt('OB_PY_FORMAT_BGR') or isfmt('OB_PY_FORMAT_MJPG_TO_BGR888'):
        require(len(body) == width * height * 3, 'COLOR_BGR_LENGTH_MISMATCH')
        img = np.frombuffer(body, dtype=np.uint8).reshape(height, width, 3).copy()
    elif isfmt('OB_PY_FORMAT_MJPG_TO_RGB888'):
        require(len(body) == width * height * 3, 'COLOR_MJPG_RGB_LENGTH_MISMATCH')
        img = cv2.cvtColor(np.frombuffer(body, dtype=np.uint8).reshape(height, width, 3),
                           cv2.COLOR_RGB2BGR)
    elif isfmt('OB_PY_FORMAT_YUYV') or isfmt('OB_PY_FORMAT_YUY2'):
        require(len(body) == width * height * 2, 'COLOR_YUYV_LENGTH_MISMATCH')
        img = cv2.cvtColor(np.frombuffer(body, dtype=np.uint8).reshape(height, width, 2),
                           cv2.COLOR_YUV2BGR_YUY2)
    elif isfmt('OB_PY_FORMAT_UYVY'):
        require(len(body) == width * height * 2, 'COLOR_UYVY_LENGTH_MISMATCH')
        img = cv2.cvtColor(np.frombuffer(body, dtype=np.uint8).reshape(height, width, 2),
                           cv2.COLOR_YUV2BGR_UYVY)
    else:
        raise RuntimeError(f'COLOR_FRAME_FORMAT_UNSUPPORTED: actual={fmt}; '
                           f'known_constants={constants}')
    require(img.shape == (height, width, 3) and img.dtype == np.uint8,
            f'COLOR_DECODE_SHAPE_FAILED: {img.shape}')
    return np.ascontiguousarray(img), fmt, len(body)


def decode_depth(frame, constants, width=640, height=400):
    require((int(frame.width()), int(frame.height())) == (width, height),
            'DEPTH_DIMENSION_MISMATCH')
    fmt = int(frame.format())
    allowed = {constants[n] for n in ('OB_PY_FORMAT_Y16', 'OB_PY_FORMAT_Y11',
                                      'OB_PY_FORMAT_Y12') if n in constants}
    require(fmt in allowed, f'DEPTH_FORMAT_UNEXPECTED: {fmt}; allowed={allowed}')
    body = safe_frame_bytes(frame)
    require(len(body) == width * height * 2,
            f'DEPTH_DATA_SIZE_MISMATCH: {len(body)}')
    raw = np.frombuffer(body, dtype='<u2').reshape(height, width).copy()
    scale = float(frame.getValueScale())
    require(math.isfinite(scale) and 0 < scale <= 100,
            f'DEPTH_SCALE_INVALID: {scale}')
    return raw, scale, fmt


def timestamp_fields(frame):
    out = {}
    for method in ('timeStampUs', 'timeStamp', 'systemTimeStamp'):
        fn = getattr(frame, method, None)
        if callable(fn):
            try:
                out[method] = int(fn())
            except Exception as exc:
                out[method + '_error'] = repr(exc)
    return out


def write_result(out, records, colors, depths, scales, meta):
    # Create output ONLY after fully collected frames and stopped SDK.
    require(not out.exists(), f'OUTPUT_ALREADY_EXISTS: {out}')
    require(out.parent.is_dir(), f'OUTPUT_PARENT_MISSING: {out.parent}')
    out.mkdir(exist_ok=False)
    for i, img in enumerate(colors, 1):
        path = out / f'rgb_{i:03d}_unaligned.png'
        require(cv2.imwrite(str(path), img), f'RGB_IMAGE_WRITE_FAILED: {path}')
    if depths:
        stack = np.stack(depths, axis=0)
        np.save(out / 'depth_raw_uint16.npy', stack, allow_pickle=False)
        np.save(out / 'depth_scales_mm_per_raw.npy', np.asarray(scales, dtype=np.float64),
                allow_pickle=False)
        last = stack[-1].astype(np.float32) * np.float32(scales[-1])
        last[(stack[-1] == 0) | (stack[-1] == 65535)] = np.nan
        np.save(out / 'last_depth_mm_nan_invalid.npy', last, allow_pickle=False)
    (out / 'capture_summary.json').write_text(json.dumps({
        'mode': meta['mode'],
        'capture_count': len(records),
        'requested_color': [640, 480, 30, meta['mjpg_format_id']],
        'requested_depth': [640, 400, 30, 11] if depths else None,
        'color_profile_index': meta['color_profile_index'],
        'depth_profile_index': meta.get('depth_profile_index'),
        'sdk_frame_pairs_not_geometric_registration': True,
        'hardware_timestamp_sync_not_validated': True,
        'rgb_depth_calibration_not_validated': True,
        'mechanical_robot_motion_authorized': False,
        'depth_scale_units': 'mm per raw depth count (vendor SDK)',
        'robot_state_and_camera_extrinsics': 'NOT_CAPTURED',
        'frames': records,
    }, indent=2, ensure_ascii=False), encoding='utf-8')
    # Last-file completion marker. A crash must not be interpreted as success.
    (out / 'CAPTURE_COMPLETE.txt').write_text(
        'CAPTURED_NOT_REGISTERED_NOT_APPROVED_FOR_ROBOT_MOTION\n', encoding='ascii')
    print('CAPTURE_SAVED:', out, flush=True)
    print('CAPTURE_SUCCESS_DIAGNOSTIC_ONLY:', len(records), flush=True)


def run(args):
    out = Path(args.out).expanduser().resolve()
    require(out.parent.is_dir(), f'OUTPUT_PARENT_MISSING: {out.parent}')
    require(not out.exists(), f'OUTPUT_ALREADY_EXISTS: {out}')
    import Pipeline
    import ObTypes
    fmts = sdk_format_constants(ObTypes)
    print('SDK_FORMAT_CONSTANTS:', fmts, flush=True)
    mjpg_fmt = fmts.get('OB_PY_FORMAT_MJPG')
    require(mjpg_fmt is not None, 'SDK_MJPG_CONSTANT_MISSING')
    require(mjpg_fmt == 5,
            f'COLOR_FORMAT_5_IS_NOT_MJPG_ON_THIS_SDK: MJPG={mjpg_fmt}')
    colors, depths, scales, records = [], [], [], []
    pipe, started = None, False
    color_keeper = depth_keeper = None
    color_idx = depth_idx = None
    try:
        pipe = Pipeline.Pipeline(None, None)
        color_keeper, color_profile, color_idx = select_profile(
            pipe, ObTypes.OB_PY_SENSOR_COLOR, ObTypes.OB_PY_STREAM_VIDEO,
            (640, 480, 30, mjpg_fmt), index_hint=1)
        config = Pipeline.Config()
        config.enableStream(color_profile)
        if args.mode == 'rgbd':
            depth_keeper, depth_profile, depth_idx = select_profile(
                pipe, ObTypes.OB_PY_SENSOR_DEPTH, ObTypes.OB_PY_STREAM_VIDEO,
                (640, 400, 30, 11), index_hint=16)
            config.enableStream(depth_profile)
        pipe.start(config, None)
        started = True
        print('CAMERA_STREAM_STARTED: mode=' + args.mode, flush=True)
        good = 0
        for attempt in range(1, args.max_waits + 1):
            frameset = pipe.waitForFrames(args.wait_ms)
            if frameset is None:
                continue
            colorframe = frameset.colorFrame()
            if colorframe is None:
                continue
            depthframe = frameset.depthFrame() if args.mode == 'rgbd' else None
            if args.mode == 'rgbd' and depthframe is None:
                continue
            good += 1
            if good <= args.warmup:
                continue
            img, color_fmt, jpg_bytes = decode_color(colorframe, fmts)
            record = {
                'index': len(records) + 1,
                'host_receive_wall_time_ns': time.time_ns(),
                'color_sdk_timestamp': timestamp_fields(colorframe),
                'color_format_id': color_fmt,
                'color_encoded_bytes': jpg_bytes,
                'depth_sdk_timestamp': None,
            }
            if depthframe is not None:
                raw, scale, depth_fmt = decode_depth(depthframe, fmts)
                depths.append(raw)
                scales.append(scale)
                valid = (raw > 0) & (raw < 65535)
                central = valid[160:240, 256:384]
                record.update({
                    'depth_sdk_timestamp': timestamp_fields(depthframe),
                    'depth_format_id': depth_fmt,
                    'depth_scale_mm_per_raw': scale,
                    'depth_global_valid_fraction': float(valid.mean()),
                    'depth_center_roi_valid_fraction': float(central.mean()),
                    'depth_center_mm': (float(raw[200, 320] * scale)
                                        if valid[200, 320] else None),
                })
            colors.append(img)
            records.append(record)
            print(f"PAIR {len(records):02d}/{args.frames}: RGB={img.shape[1]}x{img.shape[0]} "
                  + (f"Depth={record['depth_global_valid_fraction']:.2%} "
                     f"CenterROI={record['depth_center_roi_valid_fraction']:.2%} "
                     if args.mode == 'rgbd' else 'color_only ')
                  + f'color_format={color_fmt}', flush=True)
            if len(records) >= args.frames:
                break
    finally:
        if started:
            pipe.stop()
            print('CAMERA_STREAM_STOPPED', flush=True)
    require(len(records) == args.frames,
            f'INCOMPLETE_CAPTURE_NO_OUTPUT: got {len(records)}/{args.frames} '
            f'after {args.max_waits} waits; check USB/SDK logs')
    write_result(out, records, colors, depths, scales, {
        'mode': args.mode, 'mjpg_format_id': mjpg_fmt,
        'color_profile_index': color_idx, 'depth_profile_index': depth_idx,
    })
    print('NO_MANUAL_KEY_NO_ROBOT_COMMANDS_NO_RGBD_ALIGNMENT', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=['rgb', 'rgbd'], required=True)
    parser.add_argument('--out', required=True)
    parser.add_argument('--frames', type=int, default=3)
    parser.add_argument('--warmup', type=int, default=5)
    parser.add_argument('--wait-ms', type=int, default=750)
    parser.add_argument('--max-waits', type=int, default=90)
    args = parser.parse_args()
    if not (1 <= args.frames <= 30 and 0 <= args.warmup <= 30
            and args.frames + args.warmup <= args.max_waits <= 300
            and 100 <= args.wait_ms <= 2000):
        parser.error('INVALID_FRAMES_WARMUP_WAIT_LIMITS')
    try:
        run(args)
    except Exception as exc:
        print('STOP:', type(exc).__name__, str(exc), file=sys.stderr, flush=True)
        raise SystemExit(2)


if __name__ == '__main__':
    main()
