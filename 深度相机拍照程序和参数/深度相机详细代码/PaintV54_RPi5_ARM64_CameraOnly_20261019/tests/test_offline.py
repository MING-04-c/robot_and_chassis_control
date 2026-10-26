"""Offline tests. Requires numpy and opencv; no Pi, Orbbec SDK or network.

Run: python3 -B -m unittest discover -s tests -v
"""
import hashlib
import importlib
import json
import shutil
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import pi5_capture_only as module
import paint_rgbd_v5_4_camera_worker as frozen


def fake_capture(dest):
    # This constructs only disk test fixtures, NEVER camera measurements.
    bgr = np.zeros((480, 640, 3), dtype=np.uint8)
    bgr[:, :, 0] = 62
    bgr[:, :, 1] = 100
    bgr[:, :, 2] = 150
    raw = np.full((480, 640), 537, dtype=np.uint16)
    k = {'fx':454.94,'fy':454.94,'cx':328.884,'cy':246.321, 'width':640,'height':480}
    params = {
        'source':'SDK_getCameraParam_in_SAME_capture_session',
        'align_mode':'OB_PY_ALIGN_D2C_SW_MODE',
        'rgb_intrinsic':k, 'd2c_intrinsic':dict(k),
        'rgb_distortion':{'k1':0.0}, 'd2c_distortion':{'k1':0.0},
        'sdk_isMirrored':False,
    }
    meta = {'source':'ONE_SDK_FRAMESET_LEGACY_SW_D2C',
            'rgb_system_timestamp_raw':1234567,'depth_system_timestamp_raw':1234568,
            'robot_motion_authorized':False}
    frozen.save(dest, (bgr, raw, 1.0, params, k, meta))
    return dest


class CameraOfflineTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.d = Path(self.temp.name)
    def tearDown(self):
        self.temp.cleanup()

    def test_original_unchanged_and_png_exact(self):
        attempt = fake_capture(self.d/'attempt_01')
        originals = {n:module.sha256(attempt/n) for n in module.REQ_FILES + ('capture_manifest.json','CAPTURE_COMPLETE.json')}
        manifest, raw = module.verify_original_capture(attempt)
        png = module.save_depth_png(attempt, manifest, raw)
        self.assertTrue(np.array_equal(cv2.imread(str(png),cv2.IMREAD_UNCHANGED), raw))
        self.assertEqual(cv2.imread(str(png),cv2.IMREAD_UNCHANGED).dtype, np.uint16)
        for n,digest in originals.items():
            self.assertEqual(module.sha256(attempt/n), digest, 'original modified: '+n)
        self.assertEqual(json.loads((attempt/'depth_png_check.json').read_text())['depth_scale_mm_per_raw'],1.0)
        with self.assertRaises(FileExistsError):
            module.save_depth_png(attempt, manifest, raw)

    def test_depth_tamper_refused(self):
        attempt = fake_capture(self.d/'attempt_01')
        raw = np.load(str(attempt/'d2c_depth_raw.npy'), allow_pickle=False)
        raw[200,200] = 0
        with (attempt/'d2c_depth_raw.npy').open('wb') as f:
            np.save(f, raw, allow_pickle=False)
        with self.assertRaisesRegex(RuntimeError,'SHA256_MISMATCH'):
            module.verify_original_capture(attempt)

    def test_manifest_tamper_refused(self):
        attempt = fake_capture(self.d/'attempt_01')
        with (attempt/'capture_manifest.json').open('a') as f:
            f.write(' ')
        with self.assertRaisesRegex(RuntimeError,'MANIFEST_HASH_MISMATCH'):
            module.verify_original_capture(attempt)

    def test_intrinsic_mismatch_refused(self):
        attempt = fake_capture(self.d/'attempt_01')
        paramsfile = attempt/'camera_parameters_same_session.json'
        params = json.loads(paramsfile.read_text())
        params['rgb_intrinsic']['fx'] = 999.0
        paramsfile.write_text(json.dumps(params))
        manifestfile = attempt/'capture_manifest.json'
        meta = json.loads(manifestfile.read_text())
        meta['source_files_sha256']['camera_parameters_same_session.json'] = module.sha256(paramsfile)
        manifestfile.write_text(json.dumps(meta))
        donefile = attempt/'CAPTURE_COMPLETE.json'
        done = json.loads(donefile.read_text())
        done['sha256_capture_manifest'] = module.sha256(manifestfile)
        donefile.write_text(json.dumps(done))
        with self.assertRaisesRegex(RuntimeError,'INTRINSIC_USED_MISMATCH'):
            module.verify_original_capture(attempt)

    def test_elf_arch_reject_x86_and_accept_arm64(self):
        p = self.d/'libfake.so'
        header = b'\x7fELF\x02\x01' + b'\x00'*12 + (62).to_bytes(2,'little')
        p.write_bytes(header)
        with self.assertRaisesRegex(RuntimeError,'SDK_NOT_ARM64_ELF'):
            module.assert_arm64_elf(p)
        p.write_bytes(header[:18]+(183).to_bytes(2,'little'))
        module.assert_arm64_elf(p)

    def test_full_launcher_mocked_subprocess(self):
        # Aarch64 & Python3.9 preflight are mocked; real user hardware is NOT checked by this test.
        out = self.d/'captures'
        def worker(args, env, output=None, log=None):
            if output is not None:
                fake_capture(output)
            return 0
        with mock.patch.object(module.platform, 'machine', return_value='aarch64'), \
             mock.patch.object(module.sys, 'version_info',(3,9,0)), \
             mock.patch.object(module, 'sdk_environment', return_value=({},self.d)), \
             mock.patch.object(module, 'call_worker', side_effect=worker):
            code = module.main(['capture','--sdk-root',str(self.d),'--out-root',str(out)])
        self.assertEqual(code,0)
        sessions = list(out.glob('pi5_camera_v54_*'))
        self.assertEqual(len(sessions),1)
        self.assertTrue((sessions[0]/'attempt_01'/'d2c_depth_raw_uint16.png').exists())

    def test_worker_retry_then_pass(self):
        out = self.d/'captures'
        calls = []
        def worker(args, env, output=None, log=None):
            if output is None: return 0
            calls.append(str(output))
            if len(calls) == 1: return 11
            fake_capture(output)
            return 0
        with mock.patch.object(module.platform, 'machine', return_value='aarch64'), \
             mock.patch.object(module.sys, 'version_info',(3,9,0)), \
             mock.patch.object(module, 'sdk_environment', return_value=({},self.d)), \
             mock.patch.object(module, 'call_worker', side_effect=worker):
            code = module.main(['capture','--sdk-root',str(self.d),'--out-root',str(out),'--max-capture-attempts','3'])
        self.assertEqual(code,0)
        self.assertEqual(len(calls),2)

    def test_fatal_worker_stops_without_fake_success(self):
        out = self.d/'captures'
        def worker(args, env, output=None, log=None):
            return 0 if output is None else 12
        with mock.patch.object(module.platform, 'machine', return_value='aarch64'), \
             mock.patch.object(module.sys, 'version_info',(3,9,0)), \
             mock.patch.object(module, 'sdk_environment', return_value=({},self.d)), \
             mock.patch.object(module, 'call_worker', side_effect=worker):
            code = module.main(['capture','--sdk-root',str(self.d),'--out-root',str(out)])
        self.assertEqual(code,2)
        self.assertEqual(list(out.rglob('CAPTURE_COMPLETE.json')),[])

if __name__=='__main__':
    unittest.main()
