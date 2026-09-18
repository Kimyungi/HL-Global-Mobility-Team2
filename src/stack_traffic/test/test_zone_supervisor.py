import signal
from stack_traffic.zone_supervisor import ProcessGate

class Child:
    pid = 12
    def __init__(self): self.result = None
    def poll(self): return self.result
    def wait(self, timeout): self.result = 0


def test_zone_lifecycle_no_camera_outside_no_duplicate_process():
    now = [0.0]
    children, killed = [], []
    def spawn(command, **kwargs):
        assert kwargs['start_new_session']
        child = Child(); children.append(child); return child
    gate = ProcessGate(['detector'], popen=spawn, kill=lambda *x: killed.append(x), clock=lambda: now[0])
    gate.update(False); assert children == []
    gate.update(True); gate.update(True); assert len(children) == 1
    gate.update(False); assert killed == [(12, signal.SIGTERM)]
    gate.update(True); assert len(children) == 1  # old process must finish first
    now[0] = 2.1; gate.update(True); assert killed[-1][1] == signal.SIGKILL
    children[0].result = 0; gate.update(True)
    now[0] += .6; gate.update(True); assert len(children) == 2
    gate.close()


def test_crash_restarts_only_inside_zone():
    now = [0.0]
    gate = ProcessGate(['detector'], popen=lambda *a, **kw: Child(), kill=lambda *x: None, clock=lambda: now[0])
    gate.update(True); first = gate.child
    first.result = 1; gate.update(True); assert gate.child is None
    now[0] = .6; gate.update(False); assert gate.child is None
    gate.update(True); assert gate.child is not first
    gate.close()


def test_main_initializes_real_ros_node_and_forwards_parameters(monkeypatch):
    """Exercise the Humble Node API without starting a detector or camera."""
    from pathlib import Path
    import rclpy
    import yaml
    from stack_traffic import zone_supervisor

    captured = {}

    class NoHardwareGate:
        def __init__(self, command):
            captured['path'] = Path(command[-1])
            captured['params'] = yaml.safe_load(captured['path'].read_text())['/**']['ros__parameters']

        def update(self, enabled):
            assert enabled is False

        def close(self):
            captured['closed'] = True

    def spin_once(node):
        assert node.get_name() == 'traffic_zone_supervisor'
        rclpy.spin_once(node, timeout_sec=0.15)
        captured['initialized'] = True

    monkeypatch.setattr(zone_supervisor, 'ProcessGate', NoHardwareGate)
    monkeypatch.setattr(rclpy, 'spin', spin_once)
    zone_supervisor.main(args=['--ros-args',
        '-p', 'camera_backend:=oak', '-p', 'show_debug:=false',
        '-p', 'yolo.confidence:=0.185', '-p', 'test_ids:=[1, 2]'])

    assert captured['initialized'] and captured['closed']
    for key, expected in {'camera_backend': 'oak', 'show_debug': False,
                          'yolo.confidence': 0.185, 'test_ids': [1, 2]}.items():
        assert captured['params'][key] == expected
    assert not captured['path'].exists()
    assert not rclpy.ok()


def test_exit_mission_releases_shared_camera_from_approach_until_done():
    from types import SimpleNamespace
    from stack_traffic.zone_supervisor import exit_camera_requested
    for phase in range(7):
        status = SimpleNamespace(last_mission_phase=phase, LAST_IDLE=0, LAST_DONE=5)
        assert exit_camera_requested(status) == (phase not in (0,5))
