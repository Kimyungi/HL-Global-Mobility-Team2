"""Keep the perception process/camera OFF outside MGM's confirmed turn zone.

This supervisor owns no traffic decision. Each launch creates fresh detector
history; MGM alone owns signal stops and ignores heartbeat loss as a new stop.
"""
import os
import signal
import subprocess
import tempfile
import time
from pathlib import Path


def exit_camera_requested(msg):
    return msg.last_mission_phase not in (msg.LAST_IDLE, msg.LAST_DONE)


class ProcessGate:
    """Nonblocking process lifecycle; at most one detector process at a time."""
    def __init__(self, command, *, popen=subprocess.Popen, kill=os.killpg, clock=time.monotonic):
        self.command, self.popen, self.kill, self.clock = command, popen, kill, clock
        self.child = None
        self.stopping_since = None
        self.retry_at = 0.0

    def update(self, enabled):
        now = self.clock()
        if self.child is not None and self.child.poll() is not None:
            self.child = None
            self.stopping_since = None
            self.retry_at = now + .5
        if self.child is not None and not enabled and self.stopping_since is None:
            self._signal(signal.SIGTERM)
            self.stopping_since = now
        if self.child is not None and self.stopping_since is not None and now-self.stopping_since >= 2.0:
            self._signal(signal.SIGKILL)
        if enabled and self.child is None and now >= self.retry_at:
            self.child = self.popen(self.command, start_new_session=True)

    def _signal(self, sig):
        try:
            self.kill(self.child.pid, sig)
        except ProcessLookupError:
            pass

    def close(self):
        if self.child is not None:
            self._signal(signal.SIGTERM)
            try:
                self.child.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self._signal(signal.SIGKILL)
                self.child.wait(timeout=2.0)


def main(args=None):
    # Importing the supervisor does not load a model or open an OAK camera.
    import rclpy
    import yaml
    from fma_interfaces.msg import MgmState
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from std_msgs.msg import Bool

    rclpy.init(args=args)
    node = Node('traffic_zone_supervisor', automatically_declare_parameters_from_overrides=True)
    params = {name: parameter.value
              for name, parameter in node.get_parameters_by_prefix('').items()}
    handle = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', prefix='traffic_zone_', delete=False)
    with handle:
        yaml.safe_dump({'/**': {'ros__parameters': params}}, handle)
    gate = ProcessGate(['ros2', 'run', 'stack_traffic', 'stack_traffic_node',
                       '--ros-args', '--params-file', handle.name])
    state = {'enabled': False, 'exit_camera': False}
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    def received(msg):
        state['enabled'] = msg.data
        gate.update(msg.data and not state['exit_camera'])
    node.create_subscription(Bool, '/adas/traffic_zone_enabled', received, qos)
    def status(msg):
        state['exit_camera'] = exit_camera_requested(msg)
        gate.update(state['enabled'] and not state['exit_camera'])
    node.create_subscription(MgmState, '/adas/mgm_state', status, 1)
    node.create_timer(.1, lambda: gate.update(state['enabled'] and not state['exit_camera']))
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        gate.close()
        Path(handle.name).unlink(missing_ok=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
