"""V4L2 UVC control adapter; uses argv only and never opens a capture stream."""

import re
import subprocess


class V4L2Exposure:
    def __init__(self, device):
        self.device = device
        self.current = None
        self.minimum = None
        self.maximum = None
        self.step = None

    def _run(self, *options):
        result = subprocess.run(['v4l2-ctl', '-d', self.device, *options],
                                capture_output=True, text=True, timeout=1.0, check=True)
        return result.stdout

    def start(self):
        controls = self._run('--list-ctrls')
        match = re.search(r'exposure_time_absolute.*?min=(\d+) max=(\d+) step=(\d+)', controls)
        if match is None or 'auto_exposure' not in controls:
            raise RuntimeError('camera does not expose UVC absolute exposure/manual mode controls')
        self.minimum, self.maximum, self.step = map(int, match.groups())
        current = self.read()
        self._run('--set-ctrl=auto_exposure=1')
        # Freeze the reported current value when taking over from camera auto mode.
        self.set(current)
        return self.current

    def read(self):
        output = self._run('--get-ctrl=exposure_time_absolute')
        match = re.search(r'exposure_time_absolute:\s*(\d+)', output)
        if match is None:
            raise RuntimeError('could not read camera exposure')
        self.current = int(match.group(1))
        return self.current

    def set(self, value):
        value = max(self.minimum, min(self.maximum, int(value)))
        value = self.minimum + round((value - self.minimum) / self.step) * self.step
        value = max(self.minimum, min(self.maximum, value))
        self._run(f'--set-ctrl=exposure_time_absolute={value}')
        actual = self.read()
        if actual != value:
            raise RuntimeError(f'camera exposure readback {actual} differs from requested {value}')
        return actual
