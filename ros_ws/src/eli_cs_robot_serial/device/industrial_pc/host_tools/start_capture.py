#!/usr/bin/env python3
"""麒麟端：单独请求一次HK照片和一组RGB/原始深度，并由树莓派TCP回传。"""
import json
import sys
from uuid import uuid4

from robot_tcp_common import configuration, parser, request


def main():
    cli = parser(__doc__)
    cli.add_argument('--message-id', help='网络重发同一请求时复用ID，避免重复拍照')
    args = cli.parse_args()
    identity = args.message_id or uuid4().hex
    print('请求单独采集，message_id='+identity, flush=True)
    response = request(configuration(args), {
        'schema': 'paint_robot_task_request_v1', 'action': 'capture', 'message_id': identity,
    })
    print(json.dumps(response, ensure_ascii=False, indent=2))
    return 0 if response.get('ok') is True else 1


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, EOFError) as error:
        print('ERROR: '+str(error), file=sys.stderr)
        sys.exit(1)
