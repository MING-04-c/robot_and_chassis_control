#!/usr/bin/env python3
"""麒麟只读状态监视器：正常/异常两种独立TCP请求，无ROS依赖，不发送开始或恢复。"""
import json
from pathlib import Path
import sys
import time
from uuid import uuid4
from robot_tcp_common import configuration, parser, request


def write_json(path, value):
    temporary = path.with_name('.'+path.name+'.'+uuid4().hex)
    try:
        temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
        temporary.replace(path)
    finally:
        if temporary.exists():
            temporary.unlink()


def read_channel(config, channel):
    response = request(config, {'schema': 'paint_robot_status_request_v1',
                       'message_id': uuid4().hex, 'channel': channel}, retries=0)
    if response.get('ok') is not True or not isinstance(response.get(channel+'_status'), dict):
        raise ValueError(str(response.get('reason', '状态通道不可用，请同步更新树莓派与麒麟代码')))
    return response[channel+'_status']


def run(config, channel='all', once=False):
    folder = Path(config['output_dir'])/'status'
    folder.mkdir(parents=True, exist_ok=True)
    cursor = folder/'.monitor_cursor.json'
    try:
        previous = json.loads(cursor.read_text())
    except (OSError, ValueError):
        previous = {}
    if not isinstance(previous, dict):
        previous = {}
    linked, last_error = None, None
    while True:
        try:
            for name in (('normal', 'abnormal') if channel == 'all' else (channel,)):
                packet = read_channel(config, name)
                write_json(folder/(name+'.json'), packet)
                if name == 'normal':
                    print('[正常状态] %s | %s | %s | 步骤=%s/%s | 方向=%s | 剩余等待=%s秒' % (
                        packet.get('state'), packet.get('workflow'), packet.get('phase'),
                        packet.get('step_index'), packet.get('step_count'), packet.get('drive_direction'),
                        packet.get('wait_seconds_remaining')), flush=True)
                else:
                    for fault in packet.get('active_faults', []):
                        print('[异常状态] %s 来源=%s 原始码=%s 原因=%s' % (
                            fault['error_code'], fault['source'], fault.get('native_error_code'), fault['message']), flush=True)
                    if not packet.get('has_fault'):
                        print('[异常状态] 当前无结构化故障；日志异常见事件', flush=True)
                    boot = packet['monitor_id']
                    last = previous.get('sequence', 0) if previous.get('monitor_id') == boot else 0
                    fresh = [event for event in packet.get('events', [])
                             if int(event['event_id'].rsplit(':', 1)[1]) > last]
                    if fresh and last and int(fresh[0]['event_id'].rsplit(':', 1)[1]) > last+1:
                        print('[异常状态] E9902 异常事件历史超出缓冲，部分历史事件无法补收', flush=True)
                    if fresh:
                        path = folder/'events.jsonl'
                        if path.exists() and path.stat().st_size > 5*1024*1024:
                            path.replace(folder/'events.previous.jsonl')
                        with path.open('a', encoding='utf-8') as stream:
                            for event in fresh:
                                stream.write(json.dumps(event, ensure_ascii=False)+'\n')
                                print('[事件] %s %s 来源=%s 原始码=%s %s' % (
                                    event['event'], event['error_code'], event['source'],
                                    event.get('native_error_code'), event['message']), flush=True)
                    previous = {'monitor_id': boot, 'sequence': packet['sequence']}
                    write_json(cursor, previous)
            if linked is False:
                print('[恢复] E9901 TCP连接恢复，已重新读取状态', flush=True)
            linked, last_error = True, None
        except (OSError, EOFError, ValueError, KeyError) as error:
            message = str(error)
            if linked is not False or message != last_error:
                print('[异常状态] E9901 来源=kylin_tcp 原始码=%s %s' % (
                    getattr(error, 'errno', None), message), flush=True)
            linked, last_error = False, message
            fault = dict(error_code='E9901', source='kylin_tcp', message=message,
                         native_error_code=getattr(error, 'errno', None), severity='error')
            write_json(folder/'abnormal.json', dict(schema='robot_abnormal_status_v1', has_fault=True, has_abnormal=True,
                                                   active_faults=[fault], events=[], connection_lost=True))
            write_json(folder/'normal.json', dict(schema='robot_normal_status_v1', state='unknown', healthy=False,
                                                 connection_lost=True))
        if once:
            return 0 if linked else 1
        time.sleep(config['status_interval'])


def main():
    cli = parser(__doc__)
    cli.add_argument('--channel', choices=('all', 'normal', 'abnormal'), default='all')
    cli.add_argument('--once', action='store_true')
    args = cli.parse_args()
    try:
        return run(configuration(args), args.channel, args.once)
    except KeyboardInterrupt:
        return 0


if __name__ == '__main__':
    sys.exit(main())
