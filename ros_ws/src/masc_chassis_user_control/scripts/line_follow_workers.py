"""有界后台工作：一个正在执行的任务，加一个可被新任务替换的待处理槽。"""

from threading import Condition, Thread


class LatestWorker:
    def __init__(self, function, name):
        self.function = function
        self.condition = Condition()
        self.pending = self.result = None
        self.closed = False
        self.submitted = self.completed = 0
        self.thread = Thread(target=self._run, name=name, daemon=True)
        self.thread.start()

    def submit(self, value):
        with self.condition:
            if self.closed:
                raise RuntimeError('worker closed')
            self.submitted += 1
            self.pending = (self.submitted, value)
            self.condition.notify_all()
            return self.submitted

    def take(self):
        with self.condition:
            result, self.result = self.result, None
            return result

    def wait(self, ticket, timeout=2.):
        """仅启动切换/退出使用；正常控制周期不等待后台任务。"""
        with self.condition:
            if not self.condition.wait_for(lambda: self.completed >= ticket, timeout):
                raise TimeoutError('background task did not finish')

    def _run(self):
        while True:
            with self.condition:
                self.condition.wait_for(lambda: self.pending is not None or self.closed)
                if self.pending is None:
                    return
                ticket, value = self.pending
                self.pending = None
            try:
                result, error = self.function(value), None
            except Exception as exception:
                result, error = None, exception
            with self.condition:
                self.result = (ticket, result, error)
                self.completed = ticket
                self.condition.notify_all()

    def close(self):
        with self.condition:
            self.closed = True
            self.condition.notify_all()
        self.thread.join(timeout=2.)
