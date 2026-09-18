"""ROS-independent Last_mission_state; caller supplies monotonic capture times.

This module chooses a route. The MGM adapter must hold the vehicle and apply
the selected route through its acknowledged GPS handoff before resuming.
"""
from dataclasses import dataclass
from enum import Enum
import math


MODEL_CLASSES = {0: 'red_blue_red', 1: 'blue_red_red'}


class Phase(str, Enum):
    IDLE = 'IDLE'
    STOPPING = 'STOPPING'
    JUDGING = 'JUDGING'
    DONE = 'DONE'


class Direction(str, Enum):
    LEFT = 'Left'
    RIGHT = 'Right'


CLASS_DIRECTION = {0: Direction.RIGHT, 1: Direction.LEFT}
DIRECTION_ROUTE = {Direction.LEFT: '06', Direction.RIGHT: '07'}


@dataclass(frozen=True)
class Detection:
    class_id: int
    confidence: float


@dataclass(frozen=True)
class Decision:
    direction: Direction
    route_id: str
    fallback: bool
    left_votes: int
    right_votes: int


def detections_from_result(result):
    """Adapt one Ultralytics Results object; empty detections remain empty."""
    if result.boxes is None:
        return ()
    return tuple(Detection(int(class_id), float(confidence))
                 for class_id, confidence in zip(
                     result.boxes.cls.tolist(), result.boxes.conf.tolist()))


def validate_model_classes(names):
    """Reject a different checkpoint instead of silently reversing directions."""
    actual = dict(enumerate(names)) if isinstance(names, (list, tuple)) else dict(names)
    if actual != MODEL_CLASSES:
        raise ValueError(f'Expected exit model classes {MODEL_CLASSES}; got {actual}')


class LastMissionState:
    """One mission per session, three stationary seconds, one vote per frame.

    ``in_zone`` must refer to the configured, confirmed mission zone. It is
    latched on entry: a later GPS outage/zone exit cannot release the stop.
    ``reset`` is for an explicit new driving session only.
    """

    OBSERVATION_NS = 3_000_000_000
    STOPPED_SPEED_MPS = 0.001

    def __init__(self, confidence_threshold=0.5):
        if not math.isfinite(confidence_threshold) or not 0 <= confidence_threshold <= 1:
            raise ValueError('confidence_threshold must be finite and within [0, 1]')
        self.confidence_threshold = confidence_threshold
        self.reset()

    def reset(self):
        self.phase = Phase.IDLE
        self.decision = None
        self._previous_ns = None
        self._clear_window()

    def _clear_window(self):
        self.started_ns = None
        self._last_frame_ns = None
        self.left_votes = self.right_votes = 0

    @property
    def stop_required(self):
        return self.phase in (Phase.STOPPING, Phase.JUDGING)

    def update(self, now_ns, *, in_zone, speed_mps, speed_valid):
        """Call independently of inference, including when no images arrive.

        Start the timer only with actual stopped-speed evidence. Motion or
        unavailable speed restarts the stationary observation window. A clock
        rollback also discards observations and requires a new stopped window.
        """
        if self._previous_ns is not None and now_ns < self._previous_ns:
            if self.stop_required:
                self.phase = Phase.STOPPING
                self._clear_window()
            self._previous_ns = now_ns
            return self.decision
        self._previous_ns = now_ns
        if self.phase == Phase.IDLE and in_zone:
            self.phase = Phase.STOPPING
        if not self.stop_required:
            return self.decision
        stopped = (speed_valid and math.isfinite(speed_mps)
                   and abs(speed_mps) <= self.STOPPED_SPEED_MPS)
        if not stopped:
            self.phase = Phase.STOPPING
            self._clear_window()
            return None
        if self.phase == Phase.STOPPING:
            self.phase = Phase.JUDGING
            self.started_ns = now_ns
        if now_ns - self.started_ns >= self.OBSERVATION_NS:
            fallback = self.left_votes == self.right_votes
            direction = (Direction.RIGHT if self.right_votes > self.left_votes
                         else Direction.LEFT)
            self.decision = Decision(direction, DIRECTION_ROUTE[direction], fallback,
                                     self.left_votes, self.right_votes)
            self.phase = Phase.DONE
        return self.decision

    def observe(self, detections, *, captured_ns, received_ns):
        """Count the most confident valid box of a fresh frame once.

        Equal top confidence for different classes is ambiguous and contributes
        no vote. Frames captured before stopping, duplicate/out-of-order frames,
        and results arriving at/after the deadline contribute no votes.
        """
        if self.phase != Phase.JUDGING:
            return False
        deadline = self.started_ns + self.OBSERVATION_NS
        if not self.started_ns <= captured_ns <= received_ns < deadline:
            return False
        if self._last_frame_ns is not None and captured_ns <= self._last_frame_ns:
            return False
        self._last_frame_ns = captured_ns
        candidates = [d for d in detections if d.class_id in CLASS_DIRECTION
                      and math.isfinite(d.confidence)
                      and self.confidence_threshold <= d.confidence <= 1]
        if not candidates:
            return False
        confidence = max(d.confidence for d in candidates)
        directions = {CLASS_DIRECTION[d.class_id] for d in candidates
                      if d.confidence == confidence}
        if len(directions) != 1:
            return False
        if directions.pop() == Direction.LEFT:
            self.left_votes += 1
        else:
            self.right_votes += 1
        return True
