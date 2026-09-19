import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from stack_exit_decision.last_mission_state import (
    Detection, Direction, LastMissionState, Phase,
    detections_from_result, validate_model_classes,
)

SECOND = 1_000_000_000


def tick(state, seconds, *, zone=True, speed=0., valid=True):
    return state.update(int(seconds * SECOND), in_zone=zone,
                        speed_mps=speed, speed_valid=valid)


def frame(state, seconds, *detections, received=None):
    return state.observe(detections, captured_ns=int(seconds * SECOND),
                         received_ns=int((seconds if received is None else received) * SECOND))


@pytest.mark.parametrize('class_id,direction,route', [
    (0, Direction.RIGHT, '07'), (1, Direction.LEFT, '06'),
])
def test_direction_waits_full_three_seconds_after_actual_stop(class_id, direction, route):
    state = LastMissionState()
    tick(state, 0, zone=False)
    assert state.phase == Phase.IDLE
    tick(state, 1, speed=1.)
    assert state.phase == Phase.STOPPING and state.stop_required
    assert not frame(state, 1, Detection(class_id, .9))
    tick(state, 3)
    frame(state, 4, Detection(class_id, .9))
    assert tick(state, 5.999) is None
    decision = tick(state, 6)
    assert (decision.direction, decision.route_id, decision.fallback) == (direction, route, False)
    assert state.phase == Phase.DONE and not state.stop_required
    assert tick(state, 20) == decision


@pytest.mark.parametrize('detections', [(), (Detection(7, .99),),
                                      (Detection(0, .49),), (Detection(0, float('nan')),)])
def test_missing_or_invalid_detection_falls_back_to_06(detections):
    state = LastMissionState()
    tick(state, 0)
    frame(state, 1, *detections)
    decision = tick(state, 10, zone=False)
    assert decision.route_id == '06' and decision.fallback


def test_each_frame_has_one_vote_and_window_uses_majority():
    state = LastMissionState()
    tick(state, 0)
    frame(state, 1, Detection(0, .8), Detection(1, .7), Detection(1, .6))
    frame(state, 2, Detection(0, .8))
    frame(state, 2.5, Detection(1, .99))
    decision = tick(state, 10)
    assert decision.route_id == '07'
    assert (decision.left_votes, decision.right_votes) == (1, 2)


def test_tied_votes_and_ambiguous_boxes_fall_back():
    state = LastMissionState()
    tick(state, 0)
    frame(state, 1, Detection(0, .9))
    frame(state, 2, Detection(1, .9))
    assert not frame(state, 3, Detection(0, .9), Detection(1, .9))
    assert tick(state, 10).fallback


def test_old_duplicate_future_and_late_frames_do_not_change_result():
    state = LastMissionState()
    tick(state, 2)
    assert not frame(state, 1, Detection(0, .9), received=3)
    assert frame(state, 3, Detection(1, .9))
    assert not frame(state, 3, Detection(0, .9))
    assert not frame(state, 2.5, Detection(0, .9), received=4)
    assert not frame(state, 5, Detection(0, .9), received=4)
    assert not frame(state, 11, Detection(0, .9), received=12)
    assert tick(state, 12).route_id == '06'
    assert not frame(state, 13, Detection(0, .9))


@pytest.mark.parametrize('speed,valid', [(0.1, True), (0., False), (float('nan'), True)])
def test_motion_or_speed_loss_restarts_stationary_observation(speed, valid):
    state = LastMissionState()
    tick(state, 0)
    frame(state, 1, Detection(0, .9))
    tick(state, 9, speed=speed, valid=valid)
    assert state.phase == Phase.STOPPING and state.stop_required
    tick(state, 10, zone=False)
    assert tick(state, 12.999, zone=False) is None
    assert tick(state, 13, zone=False).fallback


def test_zone_loss_does_not_cancel_and_new_session_clears_decision():
    state = LastMissionState()
    tick(state, 0)
    tick(state, 1, zone=False)
    assert state.stop_required
    assert tick(state, 10, zone=False).route_id == '06'
    state.reset()
    assert state.phase == Phase.IDLE and state.decision is None
    tick(state, 11)
    frame(state, 12, Detection(0, .9))
    assert tick(state, 21).route_id == '07'


def test_clock_rollback_discards_window_and_keeps_stop():
    state = LastMissionState()
    tick(state, 10)
    frame(state, 11, Detection(0, .9))
    tick(state, 12)
    tick(state, 5)
    assert state.stop_required and state.phase == Phase.STOPPING
    tick(state, 6)
    assert tick(state, 8.999) is None
    assert tick(state, 9).fallback


def test_model_class_contract_and_results_adapter():
    validate_model_classes({0: 'red_blue_red', 1: 'blue_red_red'})
    with pytest.raises(ValueError):
        validate_model_classes({0: 'blue_red_red', 1: 'red_blue_red'})
    tensor = lambda items: SimpleNamespace(tolist=lambda: items)
    result = SimpleNamespace(boxes=SimpleNamespace(cls=tensor([0., 1.]), conf=tensor([.8, .9])))
    assert detections_from_result(result) == (Detection(0, .8), Detection(1, .9))
    assert detections_from_result(SimpleNamespace(boxes=None)) == ()
