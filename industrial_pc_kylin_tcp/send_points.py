#!/usr/bin/env python3
"""麒麟端：读取检测结果JSON，校验并发送相机坐标（毫米）。"""
import json
import hashlib
from pathlib import Path
import sys
from robot_tcp_common import configuration, parser, request
from industrial_pc_protocol import authenticate, canonical, decode, load_key, validate_result
from capture_storage import file_path


def prepare_result(value, config):
    """旧未签名文件缺ID时按内容生成稳定ID；已签名消息不修改其内容。"""
    if 'protocol' in value:
        value = authenticate(value, load_key(config['key_file']) if config.get('use_shared_key', False) else None)
    elif 'message_id' not in value:
        value = dict(value, message_id='file-'+hashlib.sha256(canonical(value)).hexdigest())
    validate_result(value)
    return value


def prepare_pixel_result(value, capture_directory):
    """将本次图像的手选像素实测点转成通讯结果；必须匹配 RGB/深度文件及集中索引。"""
    if (value.get('schema') != 'v54_depth_pixel_xyz_diagnostic_v1'
            or value.get('coordinate_frame') != 'CAMERA_PINHOLE_PRELIMINARY'):
        raise ValueError('不是支持的像素深度测试文件')
    location = Path(capture_directory).expanduser().resolve()
    folder = location.parent if location.is_file() else location
    records = []
    for root in (folder, *folder.parents):
        index = root/'.capture_index.json'
        if index.is_file():
            data = json.loads(index.read_text())
            records = [record for record in data['sessions'].values() if 'rgbd' in record and (
                file_path(root, record, 'rgb.png').resolve() == location if location.is_file() else
                folder in ((root/record['directory']).resolve(), file_path(root, record, 'rgb.png').parent.resolve()))]
            break
    if len(records) > 1:
        records = [record for record in records if
                   record['files'].get('rgb.png') == value.get('source_rgb_sha256') and
                   record['files'].get('depth.png') == value.get('source_depth_sha256')]
    if len(records) != 1:
        raise ValueError('找不到唯一的采集记录；请用 --rgb-file 指定本次 RGB PNG 文件')
    record = records[0]
    for name, field in (('rgb.png', 'source_rgb_sha256'), ('depth.png', 'source_depth_sha256')):
        actual = hashlib.sha256(file_path(root, record, name).read_bytes()).hexdigest()
        if actual != record['files'].get(name) or actual != value.get(field):
            raise ValueError(name+' 与像素测试文件来源不一致，禁止把旧点绑定到新照片')
    scale = record['rgbd']['depth_scale_mm_per_raw']
    if scale != value.get('depth_scale_mm_per_raw'):
        raise ValueError('深度比例与本次采集记录不一致')
    point = value['test_pixel_point']['xyz_mm_pinhole_preliminary']
    result = {
        'schema': 'paint_v54_pi_remote_result_v1', 'status': 'OK_3D_POINTS',
        'capture_manifest_sha256': record['rgbd']['capture_manifest_sha256'],
        'coordinate_frame': 'CAMERA_PINHOLE_PRELIMINARY', 'depth_source': 'MEASURED',
        'robot_motion_authorized': False, 'robot_base_coordinate_calibrated': False,
        'robot_command': None, 'point_source': 'MANUAL_PIXEL_DEPTH_TEST',
        'paint_semantics_verified': False,
        'points_xyz_camera_mm': [{'point_id': 'manual-pixel', 'xyz_camera_mm': point}],
    }
    result['message_id'] = 'pixel-'+hashlib.sha256(canonical(result)).hexdigest()
    validate_result(result)
    return result


def main():
    cli = parser(__doc__)
    cli.add_argument('--file', required=True, help='主机检测程序生成的结果JSON文件')
    cli.add_argument('--pixel-test', action='store_true', help='发送本次图像的手选像素深度实测点')
    cli.add_argument('--capture-dir', '--rgb-file', dest='capture_dir',
                     help='本次RGB PNG文件路径；也兼容旧采集目录')
    args = cli.parse_args()
    config = configuration(args)
    value = decode(Path(args.file).expanduser().read_bytes())
    if args.pixel_test:
        if not args.capture_dir:
            cli.error('--pixel-test 必须同时提供 --capture-dir')
        value = prepare_pixel_result(value, args.capture_dir)
    else:
        if args.capture_dir:
            cli.error('--capture-dir 需要与 --pixel-test 一起使用')
        value = prepare_result(value, config)
    kind, points, _ = validate_result(value)
    print('发送类别=%s，点数=%d，message_id=%s' % (kind, len(points), value['message_id']), flush=True)
    response = request(config, value)
    print(json.dumps(response, ensure_ascii=False, indent=2))
    return 0 if response.get('ok') is True else 1


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, EOFError) as error:
        print('ERROR: '+str(error), file=sys.stderr)
        sys.exit(1)
