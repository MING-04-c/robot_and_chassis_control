#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Raspberry Pi 5 (aarch64) camera ONLY; frozen Orbbec V5.4 worker is unmodified.

Requires an independently installed compatible Orbbec legacy Python SDK (Pipeline,
ObTypes, SW-D2C) built for aarch64 and for the selected CPython ABI. This file
ships NO SDK binaries, camera driver, U-Net, robot control, or network sender.
"""
import argparse
import datetime
import hashlib
import json
import os
import platform
import subprocess
import sys
import uuid
from pathlib import Path

HERE = Path(__file__).resolve().parent
REQ_FILES = ('rgb_001.png', 'd2c_depth_raw.npy', 'camera_parameters_same_session.json')


def sha256(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def assert_arm64_elf(path):
    """Validate ELF e_machine aarch64; does not establish runtime ABI compatibility."""
    with Path(path).open('rb') as f:
        header = f.read(20)
    if len(header) < 20 or header[:4] != b'\x7fELF':
        raise RuntimeError('SDK_NOT_ELF_BINARY: ' + str(path))
    endian_flag = header[5]
    if endian_flag not in (1, 2):
        raise RuntimeError('SDK_ELF_ENDIAN_INVALID: ' + str(path))
    machine = int.from_bytes(header[18:20], 'little' if endian_flag == 1 else 'big')
    if machine != 183:
        raise RuntimeError('SDK_NOT_ARM64_ELF: {} machine={}'.format(path, machine))


def sdk_environment(sdk_root):
    root = Path(sdk_root).expanduser().resolve(strict=True)
    python_lib = root / 'lib' / 'python_lib'
    c_lib = root / 'lib' / 'c_lib'
    for d in (python_lib, c_lib):
        if not d.is_dir() or d.is_symlink():
            raise RuntimeError('LEGACY_SDK_SUBDIRECTORY_MISSING: ' + str(d))
    found = []
    for d in (python_lib, c_lib):
        for p in d.rglob('*.so*'):
            if p.is_file():
                assert_arm64_elf(p)
                found.append(p)
    if not found:
        raise RuntimeError('NO_ARM64_SDK_SHARED_LIBRARIES_FOUND: ' + str(root))
    env = os.environ.copy()
    env['PYTHONPATH'] = os.pathsep.join(
        [str(python_lib), str(root / 'Samples'), str(HERE)] +
        ([env['PYTHONPATH']] if env.get('PYTHONPATH') else []))
    env['LD_LIBRARY_PATH'] = os.pathsep.join(
        [str(c_lib)] + ([env['LD_LIBRARY_PATH']] if env.get('LD_LIBRARY_PATH') else []))
    env['PYTHONDONTWRITEBYTECODE'] = '1'
    return env, root


def read_json(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def verify_original_capture(dest):
    """Verify frozen worker output before adding the lossless 16-bit depth PNG."""
    import cv2
    import numpy as np
    dest = Path(dest)
    if dest.is_symlink() or not dest.is_dir():
        raise RuntimeError('CAPTURE_DIRECTORY_INVALID')
    manifest_file = dest / 'capture_manifest.json'
    done_file = dest / 'CAPTURE_COMPLETE.json'
    manifest = read_json(manifest_file)
    done = read_json(done_file)
    if manifest.get('schema') != 'paint_rgbd_v5_legacy_capture_v1':
        raise RuntimeError('CAPTURE_SCHEMA_MISMATCH')
    if done.get('sha256_capture_manifest') != sha256(manifest_file):
        raise RuntimeError('CAPTURE_MANIFEST_HASH_MISMATCH')
    digests = manifest.get('source_files_sha256')
    if not isinstance(digests, dict) or set(digests) != set(REQ_FILES):
        raise RuntimeError('CAPTURE_SOURCE_FILE_SET_INVALID')
    for name in REQ_FILES:
        src = dest / name
        if src.is_symlink() or not src.is_file() or sha256(src) != digests[name]:
            raise RuntimeError('CAPTURE_SOURCE_SHA256_MISMATCH: ' + name)
    if manifest.get('robot_motion_authorized') is not False or done.get('robot_motion_authorized') is not False:
        raise RuntimeError('ROBOT_FLAG_MUST_BE_FALSE')
    k = manifest.get('intrinsic_used')
    param = read_json(dest / 'camera_parameters_same_session.json')
    if param.get('source') != 'SDK_getCameraParam_in_SAME_capture_session':
        raise RuntimeError('INTRINSICS_NOT_FROM_SAME_SESSION')
    if param.get('align_mode') != 'OB_PY_ALIGN_D2C_SW_MODE':
        raise RuntimeError('SW_D2C_MODE_NOT_CONFIRMED')
    if not isinstance(k, dict) or (k.get('width'), k.get('height')) != (640, 480):
        raise RuntimeError('INTRINSIC_GRID_MISMATCH')
    if any(k.get(f) != param.get('rgb_intrinsic', {}).get(f) for f in ('fx','fy','cx','cy')):
        raise RuntimeError('INTRINSIC_USED_MISMATCH')
    pairing = manifest.get('rgb_depth_pairing')
    if not isinstance(pairing, dict) or pairing.get('source') != 'ONE_SDK_FRAMESET_LEGACY_SW_D2C':
        raise RuntimeError('RGBD_FRAMESET_PROVENANCE_INVALID')
    scale = manifest.get('depth_scale_mm_per_raw')
    if not isinstance(scale, (int,float)) or isinstance(scale, bool) or not (0 < scale < 100):
        raise RuntimeError('DEPTH_SCALE_INVALID')
    bgr = cv2.imread(str(dest / 'rgb_001.png'), cv2.IMREAD_COLOR)
    raw = np.load(str(dest / 'd2c_depth_raw.npy'), allow_pickle=False)
    if bgr is None or bgr.shape != (480, 640, 3) or bgr.dtype != np.uint8:
        raise RuntimeError('RGB_PNG_INVALID')
    if raw.shape != (480, 640) or raw.dtype != np.uint16 or not np.any((raw > 0) & (raw < 65535)):
        raise RuntimeError('D2C_DEPTH_INVALID')
    return manifest, raw


def save_depth_png(dest, manifest, raw):
    """Additional file only; original five V5.4 files remain byte-identical."""
    import cv2
    import numpy as np
    dest = Path(dest)
    p = dest / 'd2c_depth_raw_uint16.png'
    sidecar = dest / 'depth_png_check.json'
    if p.exists() or p.is_symlink() or sidecar.exists() or sidecar.is_symlink():
        raise FileExistsError('DEPTH_PNG_OR_CHECK_ALREADY_EXISTS')
    ok, encoded = cv2.imencode('.png', raw)
    if not ok or encoded is None:
        raise RuntimeError('DEPTH_PNG_ENCODING_FAILED')
    with p.open('xb') as f:
        f.write(encoded.tobytes())
    actual = cv2.imread(str(p), cv2.IMREAD_UNCHANGED)
    if actual is None or actual.dtype != np.uint16 or not np.array_equal(actual, raw):
        raise RuntimeError('DEPTH_PNG_UINT16_ROUNDTRIP_MISMATCH')
    provenance = {
        'schema': 'orbbec_v54_extra_uint16_depth_png_v1',
        'depth_png': p.name,
        'depth_png_sha256': sha256(p),
        'original_depth_npy_sha256': manifest['source_files_sha256']['d2c_depth_raw.npy'],
        'same_grid_same_raw_uint16_values': True,
        'depth_scale_mm_per_raw': manifest['depth_scale_mm_per_raw'],
        'preview_only': False,
        'robot_motion_authorized': False,
    }
    with sidecar.open('x', encoding='utf-8') as f:
        json.dump(provenance, f, ensure_ascii=False, indent=2, allow_nan=False)
        f.write('\n')
    return p


def call_worker(args, env, output=None, log=None):
    cmd = [sys.executable, '-B', str(HERE / 'paint_rgbd_v5_4_camera_worker.py')]
    if output is None:
        cmd.append('--preflight')
    else:
        cmd.extend(['--out', str(output)])
    if log is None:
        return subprocess.run(cmd, cwd=str(HERE), env=env, check=False, timeout=30).returncode
    with Path(log).open('x', encoding='utf-8') as lf:
        try:
            return subprocess.run(cmd, cwd=str(HERE), env=env, stdout=lf, stderr=subprocess.STDOUT,
                                  check=False, timeout=120).returncode
        except subprocess.TimeoutExpired as exc:
            raise RuntimeError('SDK_WORKER_TIMEOUT_LOG=' + str(log)) from exc


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('mode', choices=('check', 'capture'), help='check: SDK import preflight; capture: one valid RGB-D Frameset')
    ap.add_argument('--sdk-root', type=Path, default=os.environ.get('PI_ORBBEC_SDK_ROOT'),
                    help='ARM64 legacy SDK python3.9 root, containing lib/python_lib and lib/c_lib')
    ap.add_argument('--out-root', type=Path, default=Path.home() / 'orbbec_v54_pi_captures')
    ap.add_argument('--max-capture-attempts', type=int, default=3)
    args = ap.parse_args(argv)
    try:
        if platform.machine().lower() not in ('aarch64', 'arm64'):
            raise RuntimeError('RASPBERRY_PI_ARM64_REQUIRED, ACTUAL=' + platform.machine())
        if sys.version_info[:2] != (3, 9):
            raise RuntimeError('ORIGINAL_LEGACY_PYTHON39_REQUIRED, ACTUAL=' + sys.version.split()[0])
        if args.sdk_root is None:
            raise RuntimeError('SDK_ROOT_REQUIRED: use --sdk-root /absolute/path/to/ARM64_legacy/python3.9')
        if not 1 <= args.max_capture_attempts <= 12:
            raise RuntimeError('MAX_CAPTURE_ATTEMPTS_MUST_BE_1_TO_12')
        env, sdk_root = sdk_environment(args.sdk_root)
        print('PI5_CAMERA_SOURCE: FROZEN_V54_WORKER_UNCHANGED', flush=True)
        print('PI5_ARCH:', platform.machine(), 'PYTHON:', sys.version.split()[0], flush=True)
        print('PI5_SDK_ROOT:', sdk_root, flush=True)
        preflight_rc = call_worker(args, env)
        if preflight_rc != 0:
            raise RuntimeError('LEGACY_SDK_PREFLIGHT_FAILED_EXIT=' + str(preflight_rc))
        print('PI5_LEGACY_SDK_PREFLIGHT_PASS_NO_CAMERA_STARTED', flush=True)
        if args.mode == 'check':
            return 0
        root = args.out_root.expanduser().resolve()
        root.mkdir(parents=True, exist_ok=True)
        if not root.is_dir():
            raise RuntimeError('CAPTURE_OUTPUT_ROOT_NOT_DIR')
        session = root / ('pi5_camera_v54_' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '_' + uuid.uuid4().hex[:10])
        session.mkdir(exist_ok=False)
        for i in range(1, args.max_capture_attempts + 1):
            dest = session / ('attempt_{:02d}'.format(i))
            log = session / ('sdk_{:02d}.log'.format(i))
            rc = call_worker(args, env, dest, log)
            if rc == 11:
                print('PI5_CAMERA_RECAPTURE: {}/{} LOG={}'.format(i,args.max_capture_attempts,log), flush=True)
                continue
            if rc != 0:
                raise RuntimeError('PI5_CAMERA_WORKER_FATAL_EXIT={} LOG={}'.format(rc,log))
            manifest, raw = verify_original_capture(dest)
            png = save_depth_png(dest, manifest, raw)
            print('PI5_CAMERA_CAPTURE_PASS_NO_UNET_NO_ROBOT', flush=True)
            print('CAPTURE_ATTEMPTS:', i, flush=True)
            print('RGB_FILE:', dest / 'rgb_001.png', flush=True)
            print('DEPTH_NPY:', dest / 'd2c_depth_raw.npy', flush=True)
            print('DEPTH_16BIT_PNG:', png, flush=True)
            print('SAME_SESSION_PARAMETERS:', dest / 'camera_parameters_same_session.json', flush=True)
            print('CAPTURE_MANIFEST:', dest / 'capture_manifest.json', flush=True)
            print('OUT:', dest, flush=True)
            print('SESSION:', session, flush=True)
            return 0
        print('PI5_CAMERA_RETRY_EXHAUSTED_NO_VALID_RGBD: SESSION=' + str(session), file=sys.stderr, flush=True)
        return 2
    except Exception as exc:
        print('PI5_CAMERA_STOP:', type(exc).__name__, str(exc), file=sys.stderr, flush=True)
        return 2


if __name__ == '__main__':
    sys.exit(main())
