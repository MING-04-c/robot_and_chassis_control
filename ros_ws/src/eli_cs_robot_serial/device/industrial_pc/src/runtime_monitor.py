"""只读运行状态归一化：项目错误码与设备原始码分开，事件有序、有界、可去重。"""
from collections import deque
from datetime import datetime, timezone
from time import monotonic
from uuid import uuid4


ERROR_CODES = {
    'E1001': '任务执行失败', 'E1002': '任务状态失联',
    'E2001': '机器人安全锁定', 'E2101': '机械臂关节任务失败',
    'E2102': '机械臂任务状态失联', 'E3001': '循迹或倒车异常',
    'E3002': '循迹状态失联', 'E3101': '倒车充电异常',
    'E4001': '底盘故障', 'E4002': '底盘通讯异常', 'E4003': '底盘状态失联',
    'E9001': 'ROS错误日志', 'E9901': '工控机TCP连接失败', 'E9902': '异常事件历史超出缓冲',
}
HEALTHY_LINE_STATES = {
    'tracking', 'disabled', 'reacquiring', 'stop_marker_intermediate', 'stop_marker_final',
    'stop_marker_approaching', 'stop_marker_missing_approaching', 'stop_marker_settling',
    'stop_marker_confirming', 'stop_marker_averaging', 'reverse_route_tracking',
    'reverse_visual_finish_tracking', 'reverse_visual_finish_aligning',
    'reverse_route_complete', 'reverse_route_ready', 'charging_dock_confirming',
}


class RuntimeMonitor:
    def __init__(self):
        self.boot_id = uuid4().hex
        self.sequence = 0
        self.samples = {}
        self.events = deque(maxlen=50)
        self.logs = deque(maxlen=20)
        self.active = {}
        self.context = None
        self.context_since = monotonic()

    def receive(self, source, value, now=None):
        if isinstance(value, dict):
            self.samples[source] = (value, monotonic() if now is None else now)

    def log_error(self, source, message, level, file='', function='', line=0, now=None):
        now = monotonic() if now is None else now
        text = str(message)[:512]
        source, file, function = str(source)[:128], str(file)[:256], str(function)[:128]
        identity = (source, text, file, line)
        if self.logs and self.logs[-1]['identity'] == identity and now-self.logs[-1]['received'] < 5.:
            return
        self.sequence += 1
        item = dict(event_id=f'{self.boot_id}:{self.sequence}', event='log', error_code='E9001',
                    source=source, message=text, native_error_code=None, level=int(level),
                    file=file, function=function, line=int(line), occurred_at=self.timestamp())
        self.logs.append(dict(identity=identity, received=now, data=item))
        self.events.append(item)

    @staticmethod
    def timestamp():
        return datetime.now(timezone.utc).isoformat()

    def snapshot(self, power=None, now=None):
        now = monotonic() if now is None else now
        def sample(name, ttl=3.):
            value, received = self.samples.get(name, ({}, None))
            age = None if received is None else max(0., now-received)
            return value, received is not None and age < ttl, age
        task, task_fresh, task_age = sample('task')
        line, line_fresh, line_age = sample('line_follow')
        arm, arm_fresh, arm_age = sample('arm')
        safety, safety_fresh, _ = sample('safety')
        inhibit, inhibit_fresh, _ = sample('inhibit')
        chassis, chassis_fresh, _ = sample('chassis', 1.)
        comm, comm_fresh, _ = sample('comm')
        faults = {}
        def fault(code, source, message, native=None, detail_code=None):
            faults[(code, source)] = dict(error_code=code, error_name=ERROR_CODES[code],
                source=source, message=str(message)[:512], native_error_code=native, detail_code=detail_code, severity='error')
        active_task = bool(task.get('active'))
        stopped_by_user = task.get('failure_reason') == 'stopped by user'
        if (task.get('failed') or task.get('state') in ('FAULT', 'FAILED')) and not stopped_by_user:
            fault('E1001', 'task_controller', task.get('failure_reason') or task.get('message') or '任务失败',
                  task.get('error_code'))
        if 'task' in self.samples and not task_fresh:
            fault('E1002', 'task_controller', '超过3秒没有新的任务状态，不能用旧进度代表当前状态')
        # 锁定反馈丢失不能等同解锁；只有新的明确false/正常反馈可解除这一项。
        if safety.get('latched') or inhibit.get('inhibited') or task.get('safety_inhibited'):
            fault('E2001', 'robot_safety', safety.get('detail') or '机器人安全锁定', safety.get('safety_mode'))
        phase = str(task.get('phase', ''))
        context = (task.get('run_id'), task.get('step_index'), phase)
        if context != self.context:
            self.context, self.context_since = context, now
        phase_stale = now-self.context_since >= 3.
        if arm.get('state') == 'FAILED':
            fault('E2101', 'arm_joint_postures', arm.get('error') or '关节任务失败', arm.get('native_error_code'))
        if active_task and phase == 'posture_wait' and not arm_fresh and phase_stale:
            fault('E2102', 'arm_joint_postures', '超过3秒没有关节任务状态')
        line_expected = active_task and task.get('action') == 'line_follow' and phase == 'follow_tracking'
        state = str(line.get('state', ''))
        if line_expected and not line_fresh and phase_stale:
            fault('E3002', 'line_follow', '循迹步骤没有新鲜运行状态')
        elif line_expected and state and state not in HEALTHY_LINE_STATES:
            fault('E3101' if state.startswith('charging_dock_') else 'E3001',
                  'line_follow', state, line.get('error_code'), state)
        if chassis.get('fault', 0):
            fault('E4001', 'chassis', '底盘原始故障位图非0', chassis['fault'])
        if comm and comm.get('state') != 4:
            fault('E4002', 'chassis_can', 'CAN通讯未连接', comm.get('fail_reason'))
        if ('comm' in self.samples and not comm_fresh or
                'chassis' in self.samples and not chassis_fresh):
            fault('E4003', 'chassis', '底盘反馈过期')
        stamp = self.timestamp()
        for key, value in faults.items():
            old = self.active.get(key)
            if old is None or old['native_error_code'] != value['native_error_code'] or old.get('detail_code') != value.get('detail_code'):
                self.sequence += 1
                self.events.append(dict(value, event='raised', occurred_at=stamp,
                                        event_id=f'{self.boot_id}:{self.sequence}'))
        for key, value in self.active.items():
            if key not in faults:
                self.sequence += 1
                self.events.append(dict(value, event='cleared', occurred_at=stamp,
                                        event_id=f'{self.boot_id}:{self.sequence}'))
        self.active = faults
        complete = (task.get('state') == 'DONE' or phase == 'capture_complete' or
                    not active_task and task.get('step_count', 0) > 0 and
                    task.get('step_index', 0) >= task['step_count'])
        running = ('exception' if faults else 'unknown' if not task_fresh else 'stopped' if stopped_by_user else 'completed' if complete else
                   'initializing' if active_task and task.get('devices_ready') is False else
                   'waiting' if phase in ('waiting_delay', 'station_wait_start', 'waiting_stationary') else
                   'running' if active_task else 'idle')
        recent_logs = [entry['data'] for entry in self.logs if now-entry['received'] < 15.]
        if recent_logs and running not in ('exception', 'unknown'):
            running = 'attention'
        base = dict(monitor_id=self.boot_id, sequence=self.sequence, updated_at=stamp)
        normal = dict(base, schema='robot_normal_status_v1', state=running,
            healthy=task_fresh and not faults and not recent_logs, workflow=task.get('workflow'), run_id=task.get('run_id'),
            active=active_task if task_fresh else None, phase=phase if task_fresh else 'unknown',
            action=task.get('action') if task_fresh else None,
            step_index=task.get('step_index'), step_count=task.get('step_count'),
            posture=task.get('posture'), drive_direction=(line.get('drive_direction') if line_fresh else None) or task.get('drive_direction'),
            line_state=state if line_fresh else 'unknown',
            charging_dock=line.get('charging_dock') if line_fresh else None,
            arm_state=arm.get('state') if arm_fresh else 'unknown',
            wait_seconds_remaining=task.get('wait_seconds_remaining'), power=power,
            freshness=dict(task=task_fresh, line_follow=line_fresh, arm=arm_fresh,
                           safety=safety_fresh or inhibit_fresh),
            ages_seconds=dict(task=task_age, line_follow=line_age, arm=arm_age))
        abnormal = dict(base, schema='robot_abnormal_status_v1', has_fault=bool(faults), has_abnormal=bool(faults or recent_logs),
            active_faults=list(faults.values()), events=list(self.events),
            recent_error_logs=recent_logs,
            error_code_namespace='robot_project_v1')
        return normal, abnormal
