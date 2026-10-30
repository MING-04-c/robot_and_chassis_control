#!/usr/bin/env python3
"""麒麟端：启动树莓派已加载任务，或发送途中继续/停止请求。"""
import json
import sys
from uuid import uuid4
from robot_tcp_common import configuration, parser, request, status


def main():
    cli = parser(__doc__)
    cli.add_argument('--action', choices=('start', 'resume', 'stop', 'status'), default='start')
    cli.add_argument('--message-id', help='重发同一命令时复用ID；同ID不会再次执行')
    args = cli.parse_args()
    config = configuration(args)
    if args.action == 'status':
        response = status(config)
    else:
        identity = args.message_id or uuid4().hex
        print('发送任务动作=%s，message_id=%s' % (args.action, identity), flush=True)
        response = request(config, {'schema': 'paint_robot_task_request_v1',
                                    'action': args.action, 'message_id': identity})
    print(json.dumps(response, ensure_ascii=False, indent=2))
    return 0 if response.get('ok') is True else 1


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, EOFError) as error:
        print('ERROR: '+str(error), file=sys.stderr)
        sys.exit(1)
