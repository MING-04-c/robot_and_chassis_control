#!/usr/bin/env python3
"""从唯一源文件导出麒麟通讯工具包；不包含密钥和现场配置数据。"""
import hashlib
import json
from pathlib import Path
import shutil
import zipfile


def main():
    root = Path(__file__).resolve().parents[1]
    device = root/'ros_ws/src/eli_cs_robot_serial/device/industrial_pc'
    output = root/'industrial_pc_kylin_tcp'
    output.mkdir(exist_ok=True)
    files = {p.name: p for p in (device/'host_tools').iterdir() if p.is_file()}
    for name in ('industrial_pc_protocol.py', 'industrial_pc_client.py', 'industrial_pc_hk_receiver.py'):
        files[name] = device/'src'/name
    files.update({
        'capture_transfer.py': root/'ros_ws/src/geminipro_camer_calibration/scripts/capture_transfer.py',
    })
    previous = root/'industrial_pc_kylin_tcp.zip'
    for name, source in files.items():
        destination = output/name
        # 不覆写部署后修改过的配置；交付源仍会带示例config。
        if name == 'config.json' and destination.exists():
            continue
        shutil.copyfile(source, destination)
        destination.chmod(0o755 if name.endswith('.py') else 0o644)
    names = sorted(files)
    manifest = {name: hashlib.sha256((output/name).read_bytes()).hexdigest() for name in names}
    (output/'SOURCE_FILES_SHA256.json').write_text(json.dumps(manifest, indent=2)+'\n')
    with zipfile.ZipFile(previous, 'w', zipfile.ZIP_DEFLATED) as archive:
        for name in [*names, 'SOURCE_FILES_SHA256.json']:
            archive.write(output/name, 'industrial_pc_kylin_tcp/'+name)
    print('导出完成：'+str(previous))


if __name__ == '__main__':
    main()
