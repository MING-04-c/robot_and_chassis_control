#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""只读检查 V5.4 camera 模式某次 attempt_XX 的真实 SDK 参数与源文件哈希。"""
import argparse
import hashlib
import json
import sys
from pathlib import Path


def sha256(file):
    h = hashlib.sha256()
    with file.open('rb') as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def read_json(file):
    return json.loads(file.read_text(encoding='utf-8'))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--capture', required=True, type=Path,
                   help='camera 命令 OUT 打印的 .../attempt_01 目录')
    a = p.parse_args()
    d = a.capture.expanduser().resolve(strict=True)
    if not d.is_dir() or not d.name.startswith('attempt_'):
        raise RuntimeError('必须指定真实 attempt_XX 目录')
    mf = d/'capture_manifest.json'
    done = d/'CAPTURE_COMPLETE.json'
    cp = d/'camera_parameters_same_session.json'
    m = read_json(mf)
    z = read_json(done)
    params = read_json(cp)
    if m.get('schema') != 'paint_rgbd_v5_legacy_capture_v1':
        raise RuntimeError('采集格式不符')
    if sha256(mf) != z.get('sha256_capture_manifest'):
        raise RuntimeError('采集清单 SHA256 不匹配')
    sources = m.get('source_files_sha256')
    if not isinstance(sources, dict) or set(sources) != {
            'rgb_001.png','d2c_depth_raw.npy','camera_parameters_same_session.json'}:
        raise RuntimeError('原始文件清单缺失或异常')
    for name, digest in sources.items():
        f = d/name
        if f.is_symlink() or not f.is_file() or sha256(f) != digest:
            raise RuntimeError('原始数据哈希不匹配: '+name)
    if m.get('robot_motion_authorized') is not False:
        raise RuntimeError('机器人安全标记缺失')
    if params.get('source') != 'SDK_getCameraParam_in_SAME_capture_session':
        raise RuntimeError('不是同一次相机会话的参数快照')

    print('V54_CAPTURE_FILES_SHA256_PASS')
    print('CAPTURE_DIR:', d)
    print('SOURCE:', params.get('source'))
    print('REQUESTED_COLOR_PROFILE: 640x480 FPS=30 MJPG(format_id=5), index=1')
    print('REQUESTED_DEPTH_PROFILE: 640x400 FPS=30 (format_id=11), index=16')
    print('ALIGN_MODE:', params.get('align_mode'))
    print('OUTPUT_RGB_AND_D2C_GRID: 640x480')
    print('DEPTH_SCALE_MM_PER_RAW:', m.get('depth_scale_mm_per_raw'))
    for key in ('rgb_intrinsic','d2c_intrinsic','rgb_distortion','d2c_distortion','sdk_isMirrored'):
        print(key+':', json.dumps(params.get(key), ensure_ascii=False, sort_keys=True))
    print('INTRINSIC_USED_FOR_XYZ:', json.dumps(m.get('intrinsic_used'),ensure_ascii=False,sort_keys=True))
    print('RGB_DEPTH_PAIRING:', json.dumps(m.get('rgb_depth_pairing'),ensure_ascii=False,sort_keys=True))
    print('CAMERA_SERIAL:', params.get('device_serial'))
    print('EXPOSURE_GAIN_WHITE_BALANCE: 未在这份已验证快照中记录，不能推断')
    print('GEOMETRIC_REGISTRATION_AND_ABSOLUTE_ACCURACY: NOT_CERTIFIED')
    print('ROBOT_MOTION_AUTHORIZED: FALSE')
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, RuntimeError) as exc:
        print('CAMERA_PARAMETERS_READONLY_STOP:', exc, file=sys.stderr)
        sys.exit(2)
