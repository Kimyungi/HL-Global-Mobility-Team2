from types import SimpleNamespace
from stack_exit_decision.last_mission_state import Detection
from stack_exit_decision.node import best_detection, detector_requested


def test_per_frame_selection_rejects_ambiguous_or_wrong_model_boxes():
    assert best_detection([]) == (-1, 0.)
    assert best_detection([Detection(0, .8), Detection(1, .9)]) == (1, .9)
    assert best_detection([Detection(0, .8), Detection(1, .8)]) == (-1, 0.)
    assert best_detection([Detection(8, 1.), Detection(0, float('nan'))]) == (-1, 0.)


def test_camera_closes_without_recent_authorized_mgm_mission():
    status = SimpleNamespace(revised_v2=True, go_authorized=True, estop_active=False,
                             last_mission_phase=2, LAST_STOPPING=1, LAST_JUDGING=2)
    assert detector_requested(status, 1, 2)
    assert not detector_requested(status, 1, 300_000_000)
    status.estop_active = True
    assert not detector_requested(status, 1, 2)
    status.estop_active = False
    status.last_mission_phase = 0
    assert not detector_requested(status, 1, 2)
