"""Start and stop a launch file as one process group. Shared by the eval tools."""
import os
import signal
import subprocess
import time

import rclpy


class Process:
    """A subprocess in its own session, so the whole tree can be signalled."""

    def __init__(self, cmd, log_path=None, env=None):
        self._log = open(log_path, 'a') if log_path else subprocess.DEVNULL
        self.proc = subprocess.Popen(cmd, stdout=self._log, stderr=subprocess.STDOUT,
                                     start_new_session=True, env=env)

    def alive(self):
        return self.proc.poll() is None

    def stop(self, timeout=20.0):
        """SIGINT the group, then SIGKILL whatever is left (Gazebo children included)."""
        try:
            pgid = os.getpgid(self.proc.pid)
        except ProcessLookupError:
            pgid = None
        if pgid is not None:
            try:
                os.killpg(pgid, signal.SIGINT)
                self.proc.wait(timeout=timeout)
            except (subprocess.TimeoutExpired, ProcessLookupError):
                pass
            try:
                os.killpg(pgid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            self.proc.wait()
        if self._log is not subprocess.DEVNULL:
            self._log.close()


def launch_sim(world, seed, log_path, launch_file='sim.launch.py', extra_args=(), env=None):
    cmd = ['ros2', 'launch', 'rescue_bringup', launch_file,
           f'world:={world}', 'headless:=true', f'seed:={seed}', *extra_args]
    return Process(cmd, log_path, env)


def spin_until(node, condition, wall_timeout, alive=lambda: True):
    """Spin until condition() holds. Returns 'ok', 'sim_died' or 'timeout'."""
    deadline = time.monotonic() + wall_timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if condition():
            return 'ok'
        if not alive():
            return 'sim_died'
    return 'timeout'
