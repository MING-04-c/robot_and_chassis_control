#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Single command launcher: U-Net industrial_ai + legacy Orbbec camera worker.
Usage: python -B run_paint_rgbd_unet_v5_4.py check|test|live|replay [options]
"""
import argparse
import hashlib
import json
from datetime import datetime
import uuid
import os
from pathlib import Path
import subprocess
import sys
ROOT=Path(__file__).resolve().parent
UNETPY=Path('/home/abc/miniconda3/envs/industrial_ai/bin/python')
MAIN=ROOT/'paint_rgbd_unet_auto_v5_4.py'
TEST=ROOT/'test_paint_rgbd_unet_v5_4.py'

# Camera-only smoke test: does not import Torch or apply U-Net to a paper board.
# This verifies that the real Orbbec stream/decoder can obtain usable RGB-D.
SDK = Path('/home/abc/orbbec_sdk_v114_unzip/OrbbecSDK_Python_v1.1.4_linux_x64_release/python3.9')
OLDPY = Path('/home/abc/miniconda3/envs/orbbec_py39/bin/python')
CAMERA_WORKER = ROOT/'paint_rgbd_v5_4_camera_worker.py'

def _sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''):
            h.update(block)
    return h.hexdigest()

def _worker_message(stderr, recoverable):
    marker='V5_4_CAMERA_RECAPTURE:' if recoverable else 'V5_4_CAMERA_FATAL:'
    for line in reversed(stderr.splitlines()):
        if line.startswith(marker):
            return line[:1500]
    return 'NO_WORKER_ERROR_MARKER_SEE_FULL_LOG'

def camera_only(argv):
    p=argparse.ArgumentParser(description='V5.2 camera-only RGB+SW-D2C smoke; no segmentation or robot')
    p.add_argument('--max-capture-attempts',type=int,default=3)
    p.add_argument('--max-waits',type=int,default=30)
    p.add_argument('--wait-ms',type=int,default=750)
    a=p.parse_args(argv)
    if not (1<=a.max_capture_attempts<=12 and 6<=a.max_waits<=120 and 100<=a.wait_ms<=2000):
        print('CAMERA_ONLY_INVALID_ARGUMENTS',file=sys.stderr)
        return 2
    # Conda commonly creates bin/python as a symlink to its own interpreter.
    # Follow it safely and verify the target is a real executable file.
    # This matches the previously real-camera-tested legacy SDK environment.
    if not OLDPY.is_file() or not os.access(OLDPY, os.X_OK):
        print('CAMERA_ONLY_PYTHON_MISSING_OR_NOT_EXECUTABLE:',OLDPY,file=sys.stderr)
        return 2
    # Keep the original anti-symlink checks for the program and SDK directories.
    for item in [CAMERA_WORKER,SDK/'lib/python_lib',SDK/'lib/c_lib']:
        if not item.exists() or item.is_symlink():
            print('CAMERA_ONLY_MISSING_OR_SYMLINK:',item,file=sys.stderr)
            return 2
    env=os.environ.copy()
    env['PYTHONPATH']=os.pathsep.join([str(ROOT),str(SDK/'lib/python_lib'),str(SDK/'Samples')]
                              +([env['PYTHONPATH']] if env.get('PYTHONPATH') else []))
    env['LD_LIBRARY_PATH']=os.pathsep.join([str(SDK/'lib/c_lib')]
                              +([env['LD_LIBRARY_PATH']] if env.get('LD_LIBRARY_PATH') else []))
    parent=ROOT/'runs/paint_rgbd_v5_4_camera_smoke'
    parent.mkdir(parents=True,exist_ok=True)
    session=None
    for _ in range(20):
        candidate=parent/('camera_v5_4_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'_'+uuid.uuid4().hex[:8])
        try:
            candidate.mkdir(exist_ok=False)
            session=candidate
            break
        except FileExistsError:
            pass
    if session is None:
        print('CAMERA_ONLY_CANNOT_CREATE_SESSION',file=sys.stderr)
        return 2
    print('CAMERA_ONLY_SESSION:',session,flush=True)
    for attempt in range(1,a.max_capture_attempts+1):
        dest=session/('attempt_%02d'%attempt)
        command=[str(OLDPY),'-B',str(CAMERA_WORKER),'--out',str(dest),
            '--warmup','5','--max-waits',str(a.max_waits),'--wait-ms',str(a.wait_ms)]
        try:
            result=subprocess.run(command,cwd=str(ROOT),env=env,capture_output=True,text=True,
                                  errors='replace',timeout=120,check=False)
        except subprocess.TimeoutExpired:
            print('CAMERA_ONLY_STOP: WORKER_TIMEOUT; session:',session,file=sys.stderr)
            return 2
        with (session/('sdk_%02d.log'%attempt)).open('x',encoding='utf-8') as f:
            f.write('EXIT_CODE={}\n--STDOUT--\n{}\n--STDERR--\n{}\n'.format(
                result.returncode,result.stdout,result.stderr))
        if result.returncode==11:
            print('CAMERA_ONLY_RECAPTURE {}/{}: {}'.format(attempt,a.max_capture_attempts,
                   _worker_message(result.stderr,True)),flush=True)
            continue
        if result.returncode!=0:
            print('CAMERA_ONLY_FATAL:',_worker_message(result.stderr,False),flush=True)
            print('CAMERA_ONLY_FULL_LOG:',session/('sdk_%02d.log'%attempt),flush=True)
            return 2
        try:
            mark=dest/'CAPTURE_COMPLETE.json'
            man=dest/'capture_manifest.json'
            if not mark.is_file() or not man.is_file():
                raise RuntimeError('CAPTURE_MARKERS_MISSING')
            meta=json.loads(man.read_text(encoding='utf-8'))
            completion=json.loads(mark.read_text(encoding='utf-8'))
            if completion.get('sha256_capture_manifest')!=_sha(man):
                raise RuntimeError('MANIFEST_HASH_INVALID')
            if meta.get('robot_motion_authorized') is not False:
                raise RuntimeError('ROBOT_FLAG_INVALID')
            for name,digest in meta.get('source_files_sha256',{}).items():
                if name not in ('rgb_001.png','d2c_depth_raw.npy','camera_parameters_same_session.json'):
                    raise RuntimeError('MANIFEST_UNEXPECTED_NAME')
                p=dest/name
                if p.is_symlink() or not p.is_file() or _sha(p)!=digest:
                    raise RuntimeError('CAPTURE_HASH_INVALID: '+name)
            if set(meta.get('source_files_sha256',{})) != {'rgb_001.png','d2c_depth_raw.npy','camera_parameters_same_session.json'}:
                raise RuntimeError('MANIFEST_FILES_INCOMPLETE')
        except (OSError,ValueError,KeyError,RuntimeError) as exc:
            print('CAMERA_ONLY_INVALID_CAPTURE:',str(exc),flush=True)
            return 2
        print('CAMERA_ONLY_RGBD_PASS_NO_UNET_NO_ROBOT')
        print('CAPTURE_ATTEMPTS:',attempt)
        print('RGB_FILE:',dest/'rgb_001.png')
        print('DEPTH_FILE:',dest/'d2c_depth_raw.npy')
        print('OUT:',dest)
        return 0
    print('CAMERA_ONLY_RETRY_EXHAUSTED_NO_ROBOT')
    print('OUT:',session)
    return 2

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('mode',choices=['check','test','live','replay','camera'])
    args,unknown=ap.parse_known_args()
    if args.mode=='camera':
        return camera_only(unknown)
    if not UNETPY.is_file():
        print('V5_LAUNCH_BLOCKED: industrial_ai Python not found',file=sys.stderr);return 2
    program=TEST if args.mode=='test' else MAIN
    if not program.is_file():
        print('V5_LAUNCH_BLOCKED: missing '+str(program),file=sys.stderr);return 2
    if args.mode=='test' and unknown:
        print('V5_LAUNCH_BLOCKED: test mode accepts no arguments',file=sys.stderr);return 2
    cmd=[str(UNETPY),'-B',str(program)]
    if args.mode!='test':cmd+=['--mode',args.mode]+unknown
    print('V5_4_START:',args.mode,'PYTHON:',UNETPY,flush=True)
    result=subprocess.run(cmd,cwd=str(ROOT),check=False,env=os.environ.copy())
    print('V5_4_EXIT_CODE=',result.returncode,flush=True)
    return result.returncode
if __name__=='__main__':sys.exit(main())
