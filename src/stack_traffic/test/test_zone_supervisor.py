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
