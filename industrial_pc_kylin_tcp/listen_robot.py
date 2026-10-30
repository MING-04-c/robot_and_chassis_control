#!/usr/bin/env python3
"""麒麟端常驻监听：海康照片、RGB-D采集包、任务状态；无需ROS或相机SDK。"""
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import select
import shutil
import socket
import sys
import tempfile
import threading
import time
from uuid import uuid4
import zipfile

import capture_transfer as rgbd
import capture_storage as storage
from industrial_pc_hk_receiver import receive_frame
from robot_tcp_common import configuration, parser, status


def atomic_json(path, value):
    temporary = path.with_name('.'+path.name+'.'+uuid4().hex)
    try:
        temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()


class RobotListener:
    def __init__(self, config):
        self.config = config
        self.output = Path(config['output_dir']).resolve()
        self.output.mkdir(parents=True, exist_ok=True)
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.channels = {name: {'state': 'starting', 'count': 0} for name in ('hk', 'rgbd', 'control')}
        self.robot_status = None
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server.bind((config['rgbd_bind'], config['rgbd_port']))
            self.server.listen(4)
            self.server.settimeout(.5)
        except Exception:
            self.server.close()
            raise
        self.rgbd_port = self.server.getsockname()[1]

    def update(self, channel, state, **values):
        with self.lock:
            self.channels[channel].update(state=state, **values)

    def saved(self, channel, path, **values):
        with self.lock:
            self.channels[channel]['count'] += 1
            self.channels[channel].update(state='saved', last_saved=str(path),
                                           received_at=datetime.now(timezone.utc).isoformat(), **values)
        print('[%s] 保存完成：%s' % (channel, path), flush=True)

    def receive_hk(self):
        """9002作为客户端长期连接；无照片时正常等待，不按空闲超时重连。"""
        while not self.stop.is_set():
            self.update('hk', 'connecting')
            try:
                with socket.create_connection((self.config['robot_host'], self.config['hk_port']),
                                              timeout=self.config['timeout']) as connection:
                    connection.settimeout(10)
                    self.update('hk', 'connected', error=None)
                    while not self.stop.is_set():
                        if not select.select([connection], [], [], .5)[0]:
                            continue
                        self.update('hk', 'receiving')
                        header, payload = receive_frame(connection)
                        capture_set = header.get('capture_set_id')
                        if not isinstance(capture_set, str) or not re.fullmatch(r'[A-Za-z0-9_.-]{1,128}', capture_set):
                            capture_set = 'legacy_'+hashlib.sha256(json.dumps(header, sort_keys=True).encode()).hexdigest()
                        destination = storage.save_hk(self.output, capture_set, header, payload)
                        self.saved('hk', destination, capture_index=header.get('capture_index', 1),
                                   capture_set_id=capture_set, error=None)
            except ConnectionRefusedError:
                # 按需拍照任务在采完前不会开放9002，持续等待发送端即可。
                self.update('hk', 'waiting_sender', error=None)
                self.stop.wait(1)
            except Exception as error:
                self.update('hk', 'reconnecting', error=str(error))
                self.stop.wait(1)

    def receive_depth_connection(self, connection):
        """完整接收、校验、解压后回ACK；失败清理临时数据并继续监听下一连接。"""
        staging = Path(tempfile.mkdtemp(prefix='.rgbd-', dir=self.output))
        try:
            if rgbd.read_exact(connection, 4) != rgbd.MAGIC:
                raise ValueError('深度通道协议标识应为RGBD')
            header = rgbd.read_json(connection)
            if not isinstance(header, dict):
                raise ValueError('深度包头必须为JSON对象')
            size = header.get('size')
            if (header.get('version') != 1 or type(size) is not int or not 1 <= size <= rgbd.MAX_ARCHIVE):
                raise ValueError('深度包版本或体积无效')
            archive_path = staging/'source.zip'
            remaining = size
            digest = hashlib.sha256()
            with archive_path.open('wb') as stream:
                while remaining:
                    block = rgbd.read_exact(connection, min(1024*1024, remaining))
                    stream.write(block)
                    digest.update(block)
                    remaining -= len(block)
                    self.update('rgbd', 'receiving', bytes_received=size-remaining, bytes_total=size)
            actual = digest.hexdigest()
            if actual != header.get('sha256'):
                raise ValueError('深度ZIP SHA256不一致')
            self.update('rgbd', 'verifying')
            count = rgbd.verify_archive(archive_path)
            data = staging/'groups'
            data.mkdir()
            # verify_archive 已拒绝路径越界、符号链接及过量解压，目标是全新的私有临时目录。
            with zipfile.ZipFile(archive_path) as archive:
                archive.extractall(data)
            summary_path = data/'dataset_summary.json'
            summary = json.loads(summary_path.read_text()) if summary_path.exists() else {}
            previous = storage.archive_record(self.output, actual)
            entries = previous
            if entries is None:
                entries = {'sessions': [], 'directories': []}
                folders = sorted(data.glob('camera_v5_4_*/attempt_*/capture_manifest.json'))
                for manifest in folders:
                    folder = manifest.parent
                    identity = (summary.get('task_run_id') if len(folders) == 1 else None)
                    identity = identity or folder.parent.name[len('camera_v5_4_'):]
                    destination = storage.save_rgbd(self.output, identity, folder, summary.get('capture_name'))
                    entries['sessions'].append(identity)
                    entries['directories'].append(str(destination.relative_to(self.output)))
                storage.archive_record(self.output, actual, entries)
            destination = self.output/entries['directories'][0]
            rgbd.write_json(connection, {'ok': True, 'groups': count, 'sha256': actual,
                                        'saved_to': str(destination), 'duplicate': previous is not None})
            self.saved('rgbd', destination, group_count=count, duplicate=previous is not None, error=None)
        except Exception as error:
            self.update('rgbd', 'error', error=str(error))
            try:
                rgbd.write_json(connection, {'ok': False, 'error': str(error)})
            except OSError:
                pass
        finally:
            if staging.exists():
                shutil.rmtree(staging)

    def receive_depth(self):
        self.update('rgbd', 'listening', endpoint='%s:%d' % (self.config['rgbd_bind'], self.rgbd_port))
        while not self.stop.is_set():
            try:
                connection, peer = self.server.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            with connection:
                connection.settimeout(30)
                self.update('rgbd', 'connected', peer=peer[0])
                self.receive_depth_connection(connection)

    def snapshot(self):
        with self.lock:
            return {'updated_at': datetime.now(timezone.utc).isoformat(),
                    'channels': {key: dict(value) for key, value in self.channels.items()},
                    'robot_status': self.robot_status}

    def run(self):
        threads = [threading.Thread(target=method, daemon=True)
                   for method in (self.receive_hk, self.receive_depth)]
        for thread in threads:
            thread.start()
        print('持续监听已启动，Ctrl+C退出；数据目录：'+str(self.output), flush=True)
        try:
            while not self.stop.is_set():
                try:
                    response = status(self.config)
                    if response.get('ok') is not True:
                        raise ValueError(str(response.get('reason', '状态查询被拒绝')))
                    with self.lock:
                        self.robot_status = response.get('robot_status')
                        self.channels['control']['count'] += 1
                    self.update('control', 'connected', error=None)
                except (OSError, EOFError, ValueError) as error:
                    self.update('control', 'reconnecting', error=str(error))
                snapshot = self.snapshot()
                atomic_json(self.output/'.listener_status.json', snapshot)
                states = ' | '.join('%s=%s(%d)' % (name, value['state'], value['count'])
                                    for name, value in snapshot['channels'].items())
                task = (snapshot['robot_status'] or {}).get('task', {})
                task_data = task.get('data') or {}
                detail = ('任务=%s phase=%s step=%s/%s fresh=%s' % (
                    task_data.get('workflow', '未收到'), task_data.get('phase', '-'),
                    task_data.get('step_index', '-'), task_data.get('step_count', '-'), task.get('fresh', False)))
                detail += ' start_service=%s' % ((snapshot['robot_status'] or {}).get('start_service_ready', False))
                detail += ' capture_service=%s' % ((snapshot['robot_status'] or {}).get('capture_service_ready', False))
                detail += ' active=%s 循迹=%s' % (task_data.get('active', '-'), task_data.get('follow_state', '-'))
                if task_data.get('failed'):
                    detail += ' 任务失败='+str(task_data.get('failure_reason', ''))
                errors = '; '.join('%s: %s' % (key, value['error'])
                                   for key, value in snapshot['channels'].items() if value.get('error'))
                print(time.strftime('[%H:%M:%S] ')+states+' | '+detail+(' | '+errors if errors else ''), flush=True)
                self.stop.wait(self.config['status_interval'])
        except KeyboardInterrupt:
            pass
        finally:
            self.stop.set()
            self.server.close()
            for thread in threads:
                thread.join(timeout=1)
            self.update('control', 'stopped')
            atomic_json(self.output/'.listener_status.json', self.snapshot())


def main():
    cli = parser(__doc__)
    args = cli.parse_args()
    RobotListener(configuration(args)).run()


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError) as error:
        print('ERROR: '+str(error), file=sys.stderr)
        sys.exit(1)
