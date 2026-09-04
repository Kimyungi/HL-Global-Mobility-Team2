import unittest
from types import SimpleNamespace

import numpy as np

from stack_traffic.node import (
    choose_yolo_tasks,
    choose_target_traffic_light,
    get_traffic_light_class_ids,
    should_freeze_signal_phase,
    should_run_yolo,
    stopline_message_values,
)


class FakeScalar:
    def __init__(self, value):
        self.value = value

    def item(self):
        return self.value


class FakeCoordinates:
    def __init__(self, coordinates):
        self.coordinates = coordinates

    def detach(self):
        return self

    def cpu(self):
        return self

    def numpy(self):
        return np.asarray(self.coordinates)


class FakeBox:
    def __init__(self, bbox, confidence, class_id=9):
        self.conf = [FakeScalar(confidence)]
        self.cls = [FakeScalar(class_id)]
        self.xyxy = [FakeCoordinates(bbox)]


class FakeResult:
    def __init__(self, boxes):
        self.boxes = boxes
        self.names = {9: "traffic light"}


class TestTargetSelection(unittest.TestCase):
    frame_shape = (480, 640, 3)

    def test_yolo_interval_starts_on_first_frame(self):
        self.assertEqual(
            [should_run_yolo(index, 2) for index in range(1, 7)],
            [True, False, True, False, True, False],
        )
        self.assertTrue(should_run_yolo(4, 1))

    def test_signal_phase_freezes_only_for_single_stop_trial(self):
        self.assertTrue(should_freeze_signal_phase(True, False, False))
        self.assertFalse(should_freeze_signal_phase(False, False, False))
        self.assertFalse(should_freeze_signal_phase(True, True, False))
        self.assertFalse(should_freeze_signal_phase(True, False, True))

    def test_yolo_tasks_never_overlap_during_red_phase(self):
        schedule = [
            choose_yolo_tasks(
                frame_index=index,
                normal_signal_interval=2,
                red_phase_signal_interval=3,
                red_phase_latched=True,
                stopline_enabled=True,
                signal_phase_frozen=False,
            )
            for index in range(1, 7)
        ]

        self.assertEqual(
            schedule,
            [
                (True, False),
                (False, True),
                (False, True),
                (True, False),
                (False, True),
                (False, True),
            ],
        )
        self.assertTrue(
            all(
                not (signal and stopline)
                for signal, stopline in schedule
            )
        )

    def test_single_stop_trial_assigns_every_red_frame_to_stopline(self):
        self.assertEqual(
            choose_yolo_tasks(
                frame_index=1,
                normal_signal_interval=2,
                red_phase_signal_interval=3,
                red_phase_latched=True,
                stopline_enabled=True,
                signal_phase_frozen=True,
            ),
            (False, True),
        )

    def test_pre_red_phase_uses_normal_signal_interval(self):
        self.assertEqual(
            [
                choose_yolo_tasks(
                    frame_index=index,
                    normal_signal_interval=2,
                    red_phase_signal_interval=3,
                    red_phase_latched=False,
                    stopline_enabled=True,
                    signal_phase_frozen=False,
                )
                for index in range(1, 5)
            ],
            [(True, False), (False, False), (True, False), (False, False)],
        )

    def test_stopline_control_detection_does_not_require_depth(self):
        runtime = SimpleNamespace(
            stable=True,
            depth=SimpleNamespace(accepted=False),
            median_camera_z_m=float("nan"),
        )

        detected, distance_m = stopline_message_values(runtime)

        self.assertTrue(detected)
        self.assertEqual(distance_m, -1.0)

    def test_valid_depth_is_kept_as_diagnostic_distance(self):
        runtime = SimpleNamespace(
            stable=True,
            depth=SimpleNamespace(accepted=True),
            median_camera_z_m=2.4,
        )

        detected, distance_m = stopline_message_values(runtime)

        self.assertTrue(detected)
        self.assertAlmostEqual(distance_m, 2.4)

    def test_traffic_light_class_ids_are_discovered_from_model_names(self):
        self.assertEqual(
            get_traffic_light_class_ids(
                {0: "person", 9: "traffic light", 10: "traffic_light"}
            ),
            [9, 10],
        )

    def _choose(
        self,
        boxes,
        previous_bbox=None,
        minimum_box_width_height_ratio=0.0,
    ):
        return choose_target_traffic_light(
            results=[FakeResult(boxes)],
            frame_shape=self.frame_shape,
            confidence_threshold=0.20,
            minimum_box_area=24,
            previous_bbox=previous_bbox,
            tracking_confidence_threshold=0.10,
            tracking_minimum_iou=0.10,
            tracking_maximum_center_shift_ratio=0.50,
            tracking_minimum_size_similarity=0.50,
            minimum_box_width_height_ratio=(
                minimum_box_width_height_ratio
            ),
        )

    def test_low_confidence_bbox_requires_existing_lock(self):
        low_confidence_box = FakeBox((102, 101, 122, 131), 0.15)

        self.assertEqual(
            self._choose([low_confidence_box]),
            (None, 0.0),
        )
        self.assertEqual(
            self._choose(
                [low_confidence_box],
                previous_bbox=(100, 100, 120, 130),
            ),
            ((102, 101, 122, 131), 0.15),
        )

    def test_locked_target_does_not_jump_to_far_high_confidence_bbox(self):
        far_box = FakeBox((300, 100, 330, 140), 0.90)

        self.assertEqual(
            self._choose(
                [far_box],
                previous_bbox=(100, 100, 120, 130),
            ),
            (None, 0.0),
        )

    def test_stopped_recovery_accepts_shifted_same_size_bbox(self):
        shifted_box = FakeBox((175, 100, 195, 130), 0.70)

        self.assertEqual(
            choose_target_traffic_light(
                results=[FakeResult([shifted_box])],
                frame_shape=self.frame_shape,
                confidence_threshold=0.20,
                minimum_box_area=24,
                previous_bbox=(100, 100, 120, 130),
                tracking_confidence_threshold=0.20,
                tracking_minimum_iou=0.10,
                tracking_maximum_center_shift_ratio=3.0,
                tracking_minimum_size_similarity=0.50,
            ),
            ((175, 100, 195, 130), 0.70),
        )

    def test_stopped_recovery_still_rejects_distant_bbox(self):
        far_box = FakeBox((300, 100, 320, 130), 0.90)

        self.assertEqual(
            choose_target_traffic_light(
                results=[FakeResult([far_box])],
                frame_shape=self.frame_shape,
                confidence_threshold=0.20,
                minimum_box_area=24,
                previous_bbox=(100, 100, 120, 130),
                tracking_confidence_threshold=0.20,
                tracking_minimum_iou=0.10,
                tracking_maximum_center_shift_ratio=3.0,
                tracking_minimum_size_similarity=0.50,
            ),
            (None, 0.0),
        )

    def test_stopped_recovery_requires_normal_detection_confidence(self):
        low_confidence_box = FakeBox((175, 100, 195, 130), 0.15)

        self.assertEqual(
            choose_target_traffic_light(
                results=[FakeResult([low_confidence_box])],
                frame_shape=self.frame_shape,
                confidence_threshold=0.20,
                minimum_box_area=24,
                previous_bbox=(100, 100, 120, 130),
                tracking_confidence_threshold=0.20,
                tracking_minimum_iou=0.10,
                tracking_maximum_center_shift_ratio=3.0,
                tracking_minimum_size_similarity=0.50,
            ),
            (None, 0.0),
        )

    def test_vertical_false_positive_is_rejected_by_aspect_ratio(self):
        vertical_box = FakeBox((300, 100, 310, 125), 0.80)
        horizontal_box = FakeBox((300, 100, 330, 115), 0.70)

        self.assertEqual(
            self._choose(
                [vertical_box, horizontal_box],
                minimum_box_width_height_ratio=0.80,
            ),
            ((300, 100, 330, 115), 0.70),
        )


if __name__ == "__main__":
    unittest.main()
