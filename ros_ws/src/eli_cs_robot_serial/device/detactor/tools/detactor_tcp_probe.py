#!/usr/bin/env python3
"""绕过ROS直接核对测厚仪TCP字节；只发原有读取命令，不修改模块或驱动机器人。"""
import argparse
import json
from pathlib import Path
import socket
import time


def receive_window(peer, seconds, phase, events, output):
    """在指定秒数内原样收集数据块，打印HEX及带转义的文本，不猜测二进制数值。"""
    end = time.monotonic() + seconds
    received = bytearray()
    while time.monotonic() < end:
        peer.settimeout(min(.2, max(.001, end-time.monotonic())))
        try:
            data = peer.recv(65536)
        except socket.timeout:
            continue
        if not data:
            raise ConnectionError('对端关闭连接')
        received.extend(data)
        record = {'phase': phase, 'time': time.time(), 'hex': data.hex(' '),
                  'text': data.decode('ascii', errors='backslashreplace')}
        events.append(record)
        output(f'RX[{phase}] {len(data)} bytes | HEX: {record["hex"]}')
        # json.dumps让NUL等不可见字符可见，ASCII数字和小数点保持原样。
        output('TEXT: '+json.dumps(record['text'], ensure_ascii=False))
    return bytes(received)


def probe(host, port, passive_seconds, response_seconds, output=print):
    """先只接收，再发送一次旧串口读取命令；返回完整原始记录和协议命中结果。"""
    events = []
    with socket.create_connection((host, port), timeout=2.) as peer:
        output(f'CONNECTED {peer.getsockname()} -> {peer.getpeername()}')
        output(f'先只接收{passive_seconds:g}秒；不发送采集命令。')
        before = receive_window(peer, passive_seconds, 'before_command', events, output)
        command = b'.d0<:>'
        output(f'TX {len(command)} bytes | HEX: {command.hex(" ")} | TEXT: .d0<:>')
        peer.sendall(command)
        events.append({'phase': 'tx', 'time': time.time(), 'hex': command.hex(' '), 'text': command.decode()})
        after = receive_window(peer, response_seconds, 'after_command', events, output)
    # 跨TCP块寻找原始文本帧；头尾可能位于不同块。
    def contains_frame(data):
        start = data.find(b'.e')
        return start >= 0 and data.find(b'<:>', start+2) >= 0
    report = {'host': host, 'port': port, 'events': events,
              'received_before_command': len(before), 'received_after_command': len(after),
              'ascii_frame_before_command': contains_frame(before),
              'ascii_frame_after_command': contains_frame(after)}
    output('结果：'+json.dumps({k: v for k, v in report.items() if k != 'events'}, ensure_ascii=False))
    if not report['ascii_frame_after_command']:
        output('本次未收到旧协议的.e…<:>文本帧；未转换为厚度。')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='169.254.199.90')
    parser.add_argument('--port', type=int, default=9001)
    parser.add_argument('--passive-seconds', type=float, default=3.)
    parser.add_argument('--response-seconds', type=float, default=5.)
    parser.add_argument('--output', type=Path, default=Path.home()/'detactor_tcp_probe.json')
    args = parser.parse_args()
    if not 1 <= args.port <= 65535 or args.passive_seconds < 0 or args.response_seconds <= 0:
        parser.error('端口或采样时间无效')
    try:
        report = probe(args.host, args.port, args.passive_seconds, args.response_seconds)
    except (OSError, ConnectionError) as error:
        parser.exit(1, f'TCP检查失败：{error}\n')
    args.output.expanduser().write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print('原始记录保存到：'+str(args.output.expanduser()))


if __name__ == '__main__':
    main()
