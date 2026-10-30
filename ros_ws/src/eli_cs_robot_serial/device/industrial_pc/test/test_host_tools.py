#!/usr/bin/env python3
"""标准库回环测试：签名请求、海康双照片、深度校验解压和重复接收。"""
import hashlib
import json
from pathlib import Path
import socket
import struct
import sys
import tempfile
import threading
import time
import unittest
import zipfile

DEVICE = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(DEVICE/'host_tools'), str(DEVICE/'src'),
               str(DEVICE.parents[2]/'geminipro_camer_calibration/scripts')]
import capture_transfer as rgbd
from industrial_pc_protocol import authenticate, envelope, read_message, write_message, validate_command
from listen_robot import RobotListener
from robot_tcp_common import request
from send_points import prepare_result, prepare_pixel_result


class HostToolsTest(unittest.TestCase):
    def test_pixel_result_requires_same_rgb_depth_and_capture_digest(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        root = Path(temporary.name)
        folder = root/'2026-10-30/19-00-00_000'
        folder.mkdir(parents=True)
        files = {}
        for name in ('rgb.png', 'depth.png'):
            payload = name.encode()
            (folder/name).write_bytes(payload)
            files[name] = hashlib.sha256(payload).hexdigest()
        (root/'.capture_index.json').write_text(json.dumps({'sessions': {'run-1': {
            'directory': str(folder.relative_to(root)), 'files': files,
            'rgbd': {'depth_scale_mm_per_raw': 1., 'capture_manifest_sha256': 'a'*64},
        }}}))
        diagnostic = {'schema': 'v54_depth_pixel_xyz_diagnostic_v1',
                      'coordinate_frame': 'CAMERA_PINHOLE_PRELIMINARY',
                      'source_rgb_sha256': files['rgb.png'], 'source_depth_sha256': files['depth.png'],
                      'depth_scale_mm_per_raw': 1.,
                      'test_pixel_point': {'xyz_mm_pinhole_preliminary': [1., 2., 300.]}}
        result = prepare_pixel_result(diagnostic, folder)
        self.assertEqual(result['capture_manifest_sha256'], 'a'*64)
        self.assertEqual(result['points_xyz_camera_mm'][0]['xyz_camera_mm'], [1., 2., 300.])
        self.assertFalse(result['paint_semantics_verified'])
        self.assertEqual(result['message_id'], prepare_pixel_result(diagnostic, folder)['message_id'])
        with self.assertRaisesRegex(ValueError, '旧点'):
            prepare_pixel_result(dict(diagnostic, source_rgb_sha256='b'*64), folder)
        (folder/'depth.png').write_bytes(b'damaged')
        with self.assertRaises(ValueError):
            prepare_pixel_result(diagnostic, folder)

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.key = b'local-test-key-with-more-than-32-bytes'
        (self.root/'key').write_bytes(self.key)
        self.config = {'robot_host': '127.0.0.1', 'control_port': 1, 'hk_port': 1,
                       'rgbd_bind': '127.0.0.1', 'rgbd_port': 0, 'timeout': 1,
                       'status_interval': .1, 'use_shared_key': True, 'key_file': str(self.root/'key'),
                       'output_dir': str(self.root/'received')}

    def listener(self):
        listener = RobotListener(self.config)
        self.addCleanup(listener.server.close)
        self.addCleanup(listener.stop.set)
        return listener

    def test_signed_request_checks_ack_identity(self):
        with socket.socket() as server:
            server.bind(('127.0.0.1', 0))
            server.listen()
            self.config['control_port'] = server.getsockname()[1]
            def respond():
                for identity in ('request-1', 'wrong-id'):
                    connection, _ = server.accept()
                    with connection:
                        payload = authenticate(read_message(connection), self.key)
                        self.assertEqual(validate_command(payload), 'status')
                        write_message(connection, envelope({'schema': 'paint_v54_pi_result_ack_v1',
                                                            'ok': True, 'message_id': identity}, self.key))
            worker = threading.Thread(target=respond, daemon=True)
            worker.start()
            payload = {'schema': 'paint_robot_status_request_v1', 'message_id': 'request-1'}
            self.assertTrue(request(self.config, payload, retries=0)['ok'])
            with self.assertRaises(ValueError):
                request(self.config, payload, retries=0)
            worker.join(2)
            self.assertFalse(worker.is_alive())

    def test_keyless_envelope_keeps_integrity_check(self):
        payload = {'schema': 'paint_robot_task_request_v1', 'action': 'capture', 'message_id': 'plain-1'}
        packet = envelope(payload, None)
        self.assertNotIn('hmac_sha256', packet)
        self.assertEqual(authenticate(packet, None), payload)
        damaged = json.loads(json.dumps(packet))
        damaged['payload']['action'] = 'start'
        with self.assertRaises(ValueError):
            authenticate(damaged, None)
        with self.assertRaises(ValueError):
            authenticate(envelope(payload, self.key), None)

    def test_legacy_demo_without_id_can_be_sent_with_stable_identity(self):
        demo = json.loads((DEVICE/'host_tools/DEMO_XYZ_800mm.json').read_text())
        demo.pop('message_id', None)
        first = prepare_result(demo, self.config)
        repeated = prepare_result(demo, self.config)
        self.assertEqual(first['message_id'], repeated['message_id'])
        self.assertNotIn('message_id', demo)
        demo['points_xyz_camera_mm'][0]['xyz_camera_mm'][0] += 1
        self.assertNotEqual(first['message_id'], prepare_result(demo, self.config)['message_id'])

    def archive(self):
        # 该夹具验证传输/哈希协议，不把合成字节宣称为相机实测。
        core = {name: b'transport-fixture-'+name.encode() for name in rgbd.CORE}
        core['camera_parameters_same_session.json'] = b'{"rgb_intrinsic":{}}'
        manifest = json.dumps({'schema': 'paint_rgbd_v5_legacy_capture_v1',
                               'robot_motion_authorized': False,
                               'source_files_sha256': {name: hashlib.sha256(body).hexdigest()
                                                       for name, body in core.items()}}).encode()
        done = json.dumps({'robot_motion_authorized': False,
                          'sha256_capture_manifest': hashlib.sha256(manifest).hexdigest()}).encode()
        path = self.root/'input.zip'
        with zipfile.ZipFile(path, 'w') as archive:
            for name, body in {**core, 'capture_manifest.json': manifest,
                               'CAPTURE_COMPLETE.json': done,
                               'd2c_depth_raw_uint16.png': b'depth-png-fixture'}.items():
                archive.writestr('camera_v5_4_test/attempt_01/'+name, body)
        return path

    def send_archive(self, listener, body, digest):
        left, right = socket.socketpair()
        left.settimeout(3)
        right.settimeout(3)
        def receive():
            with right:
                listener.receive_depth_connection(right)
        worker = threading.Thread(target=receive, daemon=True)
        worker.start()
        with left:
            left.sendall(rgbd.MAGIC)
            rgbd.write_json(left, {'version': 1, 'size': len(body), 'sha256': digest})
            left.sendall(body)
            ack = rgbd.read_json(left)
        worker.join(3)
        self.assertFalse(worker.is_alive())
        return ack

    def test_depth_repeated_packets_unpack_and_bad_packet_does_not_stop_receiver(self):
        listener = self.listener()
        body = self.archive().read_bytes()
        digest = hashlib.sha256(body).hexdigest()
        first = self.send_archive(listener, body, digest)
        self.assertTrue(first['ok'])
        directory = Path(first['saved_to'])
        self.assertEqual(directory.name, 'RGB')
        self.assertEqual(len(list(directory.glob('*_rgb.png'))), 1)
        self.assertFalse((directory/'d2c_depth_raw.npy').exists())
        self.assertEqual(len(list(directory.glob('*_depth.png'))), 1)
        self.assertFalse(self.send_archive(listener, body, '0'*64)['ok'])
        repeated = self.send_archive(listener, body, digest)
        self.assertTrue(repeated['ok'])
        self.assertTrue(repeated['duplicate'])
        self.assertEqual(first['saved_to'], repeated['saved_to'])
        self.assertEqual(len(list(directory.iterdir())), 2)
        self.assertFalse(list(listener.output.glob('.rgbd-*')))

    def test_three_image_storage_pairs_channels_and_never_overwrites(self):
        import capture_storage as storage
        folder = self.root/'unpacked'
        with zipfile.ZipFile(self.archive()) as archive:
            archive.extractall(folder)
        source = folder/'camera_v5_4_test/attempt_01'
        for depth_first in (True, False):
            root = self.root/('depth-first' if depth_first else 'hk-first')
            header = {'version': 1, 'capture_index': 1, 'capture_name': '2026.10.30.20.29.18_649',
                      'payload_sha256': hashlib.sha256(b'hk-photo').hexdigest()}
            if depth_first:
                storage.save_rgbd(root, 'same-task', source, header['capture_name'])
            image = storage.save_hk(root, 'same-task', header, b'hk-photo')
            saved = storage.save_rgbd(root, 'same-task', source, header['capture_name'])
            self.assertEqual(image.parent.name, 'hk')
            self.assertEqual(saved.name, 'RGB')
            self.assertEqual(image.parent.parent, saved.parent)
            self.assertEqual(saved.parent.name, '2026.10.30.20.29')
            self.assertEqual(sorted(p.name for p in saved.parent.iterdir()), ['RGB', 'hk'])
            self.assertEqual(sorted(p.name for p in saved.iterdir()),
                             ['20.29.18_649_depth.png', '20.29.18_649_rgb.png'])
            self.assertEqual((saved/'20.29.18_649_depth.png').read_bytes(), (source/'d2c_depth_raw_uint16.png').read_bytes())
            with self.assertRaises(ValueError):
                storage.save_hk(root, 'same-task', header, b'corrupted-hk-photo')
            other = storage.save_hk(root, 'different-task', header, b'hk-photo')
            self.assertEqual(other.parent, image.parent)
            self.assertNotEqual(other.name, image.name)
            self.assertEqual(other.name, '20.29.18_649_02_hk.jpg')
            second = storage.save_rgbd(root, 'different-task', source,
                                       storage.capture_label(root, 'different-task'))
            self.assertEqual(second, saved)
            self.assertEqual(len(list(saved.iterdir())), 4)
            self.assertEqual(storage.save_hk(root, 'same-task', header, b'hk-photo'), image)
            with storage.index(root) as metadata:
                record = metadata['sessions']['same-task']
            diagnostic = {'schema': 'v54_depth_pixel_xyz_diagnostic_v1',
                          'coordinate_frame': 'CAMERA_PINHOLE_PRELIMINARY',
                          'source_rgb_sha256': record['files']['rgb.png'],
                          'source_depth_sha256': record['files']['depth.png'],
                          'depth_scale_mm_per_raw': record['rgbd']['depth_scale_mm_per_raw'],
                          'test_pixel_point': {'xyz_mm_pinhole_preliminary': [1., 2., 300.]}}
            result = prepare_pixel_result(diagnostic, storage.file_path(root, record, 'rgb.png'))
            self.assertEqual(result['capture_manifest_sha256'], record['rgbd']['capture_manifest_sha256'])
            with self.assertRaisesRegex(ValueError, '唯一'):
                prepare_pixel_result(diagnostic, saved)  # 相同图像的两次采集需按文件选择。

    def test_old_index_can_be_read_and_copied_without_changing_originals(self):
        import capture_storage as storage
        old = self.root/'old'
        directory = old/'2026-10-30/20-29-18_649'
        directory.mkdir(parents=True)
        hashes = {}
        for name in ('hk.jpg', 'rgb.png', 'depth.png'):
            payload = ('legacy-'+name).encode()
            (directory/name).write_bytes(payload)
            hashes[name] = hashlib.sha256(payload).hexdigest()
        record = {'directory': str(directory.relative_to(old)), 'files': hashes,
                  'rgbd': {'capture_manifest_sha256': 'a'*64}, 'hk': {'hk.jpg': {'version': 1}}}
        (old/'.capture_index.json').write_text(json.dumps({'version': 1,
            'sessions': {'old-task': record}, 'archives': {'b'*64: {
                'sessions': ['old-task'], 'directories': [record['directory']]}}}))
        self.assertIsNotNone(storage.archive_record(old, 'b'*64))
        output = self.root/'organized'
        names = storage.organize_legacy(old, output)
        self.assertEqual(names, ['2026.10.30.20.29/RGB/20.29.18_649_depth.png',
                                 '2026.10.30.20.29/RGB/20.29.18_649_rgb.png',
                                 '2026.10.30.20.29/hk/20.29.18_649_hk.jpg'])
        self.assertTrue((directory/'depth.png').is_file())
        self.assertEqual((output/names[0]).read_bytes(), (directory/'depth.png').read_bytes())
        self.assertIsNotNone(storage.archive_record(output, 'b'*64))

    def test_hik_two_photos_go_to_separate_directories(self):
        with socket.socket() as server:
            server.bind(('127.0.0.1', 0))
            server.listen()
            self.config['hk_port'] = server.getsockname()[1]
            listener = self.listener()
            worker = threading.Thread(target=listener.receive_hk, daemon=True)
            worker.start()
            connection, _ = server.accept()
            with connection:
                for index in (1, 2):
                    image = ('jpeg-fixture-%d' % index).encode()
                    header = {'version': 1, 'type': 'image', 'encoding': 'jpeg',
                              'payload_length': len(image), 'capture_set_id': 'session_000001',
                              'capture_index': index, 'sequence': index, 'stamp_ns': index}
                    data = json.dumps(header).encode()
                    # 显式拆分前缀，验证TCP半包读取。
                    connection.sendall(b'MI')
                    connection.sendall(b'MG'+struct.pack('!I', len(data))+data+image)
                deadline = time.monotonic()+3
                while listener.snapshot()['channels']['hk']['count'] < 2 and time.monotonic() < deadline:
                    time.sleep(.02)
                listener.stop.set()
            worker.join(2)
            self.assertFalse(worker.is_alive())
            import capture_storage
            with capture_storage.index(listener.output) as entries:
                folder = listener.output/entries['sessions']['session_000001']['directory']
            self.assertEqual(len(list((folder/'hk').glob('*_hk.jpg'))), 1)
            self.assertEqual(len(list((folder/'hk').glob('*_hk_02.jpg'))), 1)
            self.assertEqual(len(list((folder/'hk').iterdir())), 2)

    def test_task_final_single_hk_is_saved_without_waiting_for_second_photo(self):
        with socket.socket() as server:
            server.bind(('127.0.0.1', 0))
            server.listen()
            server.settimeout(3)
            self.config['hk_port'] = server.getsockname()[1]
            listener = self.listener()
            worker = threading.Thread(target=listener.receive_hk, daemon=True)
            worker.start()
            connection, _ = server.accept()
            try:
                with connection:
                    payload = b'final-hk-jpeg-fixture'
                    header = {'version': 1, 'type': 'image', 'encoding': 'jpeg',
                              'sequence': 1, 'stamp_ns': 1, 'payload_length': len(payload),
                              'capture_set_id': 'final-task-123', 'capture_index': 1,
                              'grouping': 'task_final_one_hk'}
                    body = json.dumps(header).encode()
                    connection.sendall(b'MIMG'+struct.pack('!I', len(body))+body+payload)
                    deadline = time.monotonic()+3
                    while listener.snapshot()['channels']['hk']['count'] < 1 and time.monotonic() < deadline:
                        time.sleep(.02)
                    import capture_storage
                    with capture_storage.index(listener.output) as entries:
                        record = entries['sessions']['final-task-123']
                    folder = listener.output/record['directory']
                    self.assertEqual((capture_storage.file_path(listener.output, record, 'hk.jpg')).read_bytes(), payload)
                    self.assertEqual(record['hk']['hk.jpg']['grouping'], 'task_final_one_hk')
                    self.assertEqual(len(list((folder/'hk').iterdir())), 1)
                    self.assertEqual(listener.snapshot()['channels']['hk']['count'], 1)
            finally:
                listener.stop.set()
                worker.join(2)
            self.assertFalse(worker.is_alive())

    def test_continuous_listener_reports_status_and_receives_multiple_connections(self):
        with socket.socket() as control:
            control.bind(('127.0.0.1', 0))
            control.listen()
            control.settimeout(.1)
            self.config['control_port'] = control.getsockname()[1]
            listener = self.listener()
            def respond():
                while not listener.stop.is_set():
                    try:
                        connection, _ = control.accept()
                    except socket.timeout:
                        continue
                    with connection:
                        payload = authenticate(read_message(connection), self.key)
                        write_message(connection, envelope({
                            'schema': 'paint_v54_pi_result_ack_v1', 'ok': True,
                            'message_id': payload['message_id'],
                            'robot_status': {'task': {'fresh': True, 'data': {
                                'workflow': 'test', 'phase': 'ready', 'step_index': 0, 'step_count': 1}}},
                        }, self.key))
            responder = threading.Thread(target=respond, daemon=True)
            responder.start()
            worker = threading.Thread(target=listener.run, daemon=True)
            worker.start()
            body = self.archive().read_bytes()
            digest = hashlib.sha256(body).hexdigest()
            try:
                for expected_ok, checksum in ((False, '0'*64), (True, digest), (True, digest)):
                    with socket.create_connection(('127.0.0.1', listener.rgbd_port), timeout=3) as connection:
                        connection.settimeout(3)
                        connection.sendall(rgbd.MAGIC)
                        rgbd.write_json(connection, {'version': 1, 'size': len(body), 'sha256': checksum})
                        connection.sendall(body)
                        self.assertEqual(rgbd.read_json(connection)['ok'], expected_ok)
                deadline = time.monotonic()+3
                while time.monotonic() < deadline:
                    report = listener.output/'.listener_status.json'
                    if report.exists() and json.loads(report.read_text())['channels']['rgbd']['count'] == 2:
                        break
                    time.sleep(.02)
                saved = json.loads(report.read_text())
                self.assertEqual(saved['channels']['rgbd']['count'], 2)
                self.assertTrue(saved['robot_status']['task']['fresh'])
            finally:
                listener.stop.set()
                worker.join(3)
                responder.join(2)
            self.assertFalse(worker.is_alive())


if __name__ == '__main__':
    unittest.main()
