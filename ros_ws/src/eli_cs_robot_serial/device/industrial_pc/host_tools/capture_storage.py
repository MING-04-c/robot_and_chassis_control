"""照片归档：年月日时分/hk、RGB，参数和配对信息保存在后台索引。"""
from contextlib import contextmanager
from datetime import datetime, timedelta, timezone
import fcntl
import hashlib
import json
from pathlib import Path
import re
import threading
from uuid import uuid4

_LOCK = threading.RLock()
TZ = timezone(timedelta(hours=8))


def atomic_json(path, value):
    path = Path(path)
    temporary = path.with_name('.'+path.name+'.'+uuid4().hex)
    try:
        temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


@contextmanager
def index(root):
    root = Path(root)
    root.mkdir(parents=True, exist_ok=True)
    with _LOCK, (root/'.capture_index.lock').open('a') as lock:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX)
        path = root/'.capture_index.json'
        data = json.loads(path.read_text()) if path.exists() else {'version': 2, 'sessions': {}, 'archives': {}}
        try:
            yield data
            atomic_json(path, data)
        finally:
            fcntl.flock(lock.fileno(), fcntl.LOCK_UN)


def digest(path):
    result = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            result.update(block)
    return result.hexdigest()


def session(root, data, identity, stamp_ns=None, capture_name=None):
    if not isinstance(identity, str) or not re.fullmatch(r'[A-Za-z0-9_.-]{1,160}', identity):
        raise ValueError('采集编号无效')
    if identity in data['sessions']:
        return data['sessions'][identity]
    instant = datetime.fromtimestamp(stamp_ns/1e9, TZ) if stamp_ns else datetime.now(TZ)
    suffix_text = ''
    if isinstance(capture_name, str):
        modern = re.fullmatch(r'(\d{4}(?:\.\d{2}){5}_\d{3})(_\d+)?', capture_name)
        legacy = re.fullmatch(r'(\d{4}-\d{2}-\d{2}/\d{2}-\d{2}-\d{2}_\d{3})(_\d+)?', capture_name)
        if modern or legacy:
            matched = modern or legacy
            pattern = '%Y.%m.%d.%H.%M.%S_%f' if modern else '%Y-%m-%d/%H-%M-%S_%f'
            instant = datetime.strptime(matched[1], pattern).replace(tzinfo=TZ)
            suffix_text = matched[2] or ''
    directory = instant.strftime('%Y.%m.%d.%H.%M')
    prefix = instant.strftime('%H.%M.%S_')+f'{instant.microsecond//1000:03d}'
    label = instant.strftime('%Y.%m.%d.%H.%M.%S_')+f'{instant.microsecond//1000:03d}'+suffix_text
    candidate, suffix = label, 1
    used = {entry.get('capture_name', entry['directory']) for entry in data['sessions'].values()}
    while (candidate in used or any(any((Path(root)/directory/camera).glob(
            prefix+suffix_text+candidate[len(label):]+'_*')) for camera in ('hk', 'RGB'))):
        suffix += 1
        candidate = label+f'_{suffix:02d}'
    for camera in ('hk', 'RGB'):
        (Path(root)/directory/camera).mkdir(parents=True, exist_ok=True)
    record = {'layout': 'minute_camera_v2', 'directory': directory,
              'capture_name': candidate, 'prefix': prefix+suffix_text+candidate[len(label):],
              'files': {}, 'paths': {}}
    data['version'] = 2
    data['sessions'][identity] = record
    return record


def file_path(root, record, name):
    """逻辑图像名映射为实际路径；兼容原日期/时间三图记录，不改动旧数据。"""
    relative = record.get('paths', {}).get(name)
    if relative is None:
        if record.get('layout') == 'minute_camera_v2':
            camera = 'hk' if name.startswith('hk') else 'RGB'
            relative = str(Path(record['directory'])/camera/(record['prefix']+'_'+name))
        else:
            relative = str(Path(record['directory'])/name)
    if Path(relative).is_absolute() or '..' in Path(relative).parts:
        raise ValueError('归档路径不得越过接收目录')
    return Path(root)/relative


def capture_label(root, identity):
    """取得跨通道共享的采集名称；它是配对标识，不是额外的目录层级。"""
    with index(root) as data:
        record = data['sessions'][identity]
        return record.get('capture_name', record['directory'])


def store_file(root, record, name, source=None, payload=None):
    destination = file_path(root, record, name)
    destination.parent.mkdir(parents=True, exist_ok=True)
    checksum = digest(source) if source is not None else hashlib.sha256(payload).hexdigest()
    if destination.exists():
        if digest(destination) != checksum:
            raise ValueError('同一采集编号的图像不同或已损坏，拒绝覆盖：'+str(destination))
    else:
        temporary = destination.with_name('.'+name+'.'+uuid4().hex)
        try:
            with temporary.open('xb') as output:
                if source is None:
                    output.write(payload)
                else:
                    with Path(source).open('rb') as stream:
                        for block in iter(lambda: stream.read(1024*1024), b''):
                            output.write(block)
            temporary.replace(destination)
        finally:
            temporary.unlink(missing_ok=True)
    record['files'][name] = checksum
    if record.get('layout') == 'minute_camera_v2':
        record['paths'][name] = str(destination.relative_to(root))
    return destination


def save_hk(root, identity, header, payload):
    if header.get("payload_sha256") and hashlib.sha256(payload).hexdigest() != header["payload_sha256"]:
        raise ValueError("HK照片SHA256校验失败")
    with index(root) as data:
        record = session(root, data, identity, header.get('stamp_ns'), header.get('capture_name'))
        number = header.get('capture_index', 1)
        extension = '.jpg' if header.get('version', 1) == 1 else '.raw'
        name = ('hk' if number == 1 else 'hk_02')+extension
        destination = store_file(root, record, name, payload=payload)
        record.setdefault('hk', {})[name] = header
        return destination


def save_rgbd(root, identity, folder, capture_name=None):
    folder = Path(folder)
    manifest_path = folder/'capture_manifest.json'
    manifest = json.loads(manifest_path.read_text())
    pairing = manifest.get('rgb_depth_pairing', {})
    stamp = pairing.get('rgb_system_timestamp_raw')
    stamp_ns = int(stamp*1000000) if isinstance(stamp, (int, float)) and stamp > 0 else None
    parameters = json.loads((folder/'camera_parameters_same_session.json').read_text())
    for name in ('rgb_001.png', 'd2c_depth_raw_uint16.png'):
        if not (folder/name).is_file():
            raise ValueError('采集缺少图像：'+name)
    with index(root) as data:
        record = session(root, data, identity, stamp_ns, capture_name)
        store_file(root, record, 'rgb.png', source=folder/'rgb_001.png')
        store_file(root, record, 'depth.png', source=folder/'d2c_depth_raw_uint16.png')
        # 参数、原采集清单哈希仍用于后续三维检测点回传，不与图像一起铺满目录。
        record['rgbd'] = {'capture_manifest_sha256': digest(manifest_path),
                          'depth_scale_mm_per_raw': manifest.get('depth_scale_mm_per_raw'),
                          'camera_parameters': parameters,
                          'capture_manifest': manifest}
        return file_path(root, record, 'rgb.png').parent


def archive_record(root, checksum, entries=None):
    with index(root) as data:
        existing = data['archives'].get(checksum)
        if existing:
            for identity in existing['sessions']:
                record = data['sessions'][identity]
                for name, expected in record['files'].items():
                    if digest(file_path(root, record, name)) != expected:
                        raise ValueError('已归档照片损坏：'+name)
            return existing
        if entries is not None:
            data['archives'][checksum] = entries
        return None


def organize_legacy(source, output):
    """将旧归档复制成时分/hk、RGB；不修改或删除原始数据。"""
    source, output = Path(source).resolve(), Path(output).resolve()
    if source == output or source in output.parents or output.exists():
        raise ValueError('输出必须是源目录以外、尚不存在的新目录')
    output.mkdir(parents=True)
    old_index = source/'.capture_index.json'
    if old_index.is_file():
        previous = json.loads(old_index.read_text())
        with index(output) as data:
            for identity, old in previous['sessions'].items():
                record = session(output, data, identity, capture_name=old.get('capture_name', old['directory']))
                for name, checksum in old['files'].items():
                    photo = file_path(source, old, name)
                    if digest(photo) != checksum:
                        raise ValueError('旧归档照片校验失败：'+str(photo))
                    store_file(output, record, name, source=photo)
                for field in ('hk', 'rgbd'):
                    if field in old:
                        record[field] = old[field]
            for checksum, entry in previous.get('archives', {}).items():
                data['archives'][checksum] = {
                    **entry, 'directories': [str(file_path(output, data['sessions'][identity], 'rgb.png').parent.relative_to(output))
                                             for identity in entry['sessions']],
                }
        return sorted(str(p.relative_to(output)) for p in output.glob('*/*/*') if p.is_file())
    for metadata in sorted((source/'hk').glob('**/*.json')):
        info = json.loads(metadata.read_text())
        header = info.get('header')
        photo = metadata.with_suffix('.jpg')
        if not header or not photo.is_file():
            continue
        if info.get('sha256') and digest(photo) != info['sha256']:
            raise ValueError('旧HK照片校验失败：'+str(photo))
        identity = header.get('capture_set_id') or 'legacy_'+digest(photo)
        save_hk(output, identity, header, photo.read_bytes())
    archives = sorted((source/'rgbd').iterdir()) if (source/'rgbd').exists() else []
    for archive_dir in archives:
        if not archive_dir.is_dir():
            continue
        summary_path = archive_dir/'groups/dataset_summary.json'
        summary = json.loads(summary_path.read_text()) if summary_path.exists() else {}
        entries = {'sessions': [], 'directories': []}
        for manifest in sorted(archive_dir.glob('groups/camera_v5_4_*/attempt_*/capture_manifest.json')):
            folder = manifest.parent
            identity = summary.get('task_run_id') or folder.parent.name[len('camera_v5_4_'):]
            destination = save_rgbd(output, identity, folder, summary.get('capture_name'))
            entries['sessions'].append(identity)
            entries['directories'].append(str(destination.relative_to(output)))
        if entries['sessions'] and (archive_dir/'source.zip').is_file():
            archive_record(output, digest(archive_dir/'source.zip'), entries)
    return sorted(str(p.relative_to(output)) for p in output.glob('*/*/*') if p.is_file())


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='将旧接收数据复制成年.月.日.时.分/hk、RGB目录，原文件保留')
    parser.add_argument('--source', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    for name in organize_legacy(args.source, args.output):
        print(name)
