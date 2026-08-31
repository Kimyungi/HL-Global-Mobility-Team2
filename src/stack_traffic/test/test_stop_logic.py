import math
import unittest

from stack_traffic.logic import (
    bbox_iou,
    camera_poll_timed_out,
    classify_color_ratios,
    combine_stopline_proximity,
    frame_bbox_to_roi,
    is_red_clear_confirmed,
    is_stopline_approaching,
    is_stopline_y_approaching,
    normalized_roi_to_bbox,
    robust_nonnegative_median,
    roi_bbox_to_frame,
    select_horizontal_roi_tile,
    select_tracking_candidate,
    should_accept_anchored_green,
    should_clear_visual_track,
    should_record_color_vote,
    update_red_phase_latch,
    update_stop_latch,
)


class TestTrafficStopLogic(unittest.TestCase):
    def test_visual_track_survives_yolo_misses_with_valid_template(self):
        self.assertFalse(
            should_clear_visual_track(
                yolo_missed_frames=8,
                template_failed_frames=0,
                maximum_yolo_misses=5,
                maximum_template_failures=3,
            )
        )

    def test_visual_track_survives_brief_template_dropout(self):
        self.assertFalse(
            should_clear_visual_track(
                yolo_missed_frames=5,
                template_failed_frames=2,
                maximum_yolo_misses=5,
                maximum_template_failures=3,
            )
        )

    def test_visual_track_clears_after_both_trackers_fail(self):
        self.assertTrue(
            should_clear_visual_track(
                yolo_missed_frames=5,
                template_failed_frames=3,
                maximum_yolo_misses=5,
                maximum_template_failures=3,
            )
        )

    def test_wide_search_roi_scans_center_left_right_tiles(self):
        search_roi = (102, 36, 1178, 360)

        tiles = [
            select_horizontal_roi_tile(search_roi, 512, None, index)
            for index in range(3)
        ]

        self.assertEqual(tiles[0], (384, 36, 896, 360))
        self.assertEqual(tiles[1], (102, 36, 614, 360))
        self.assertEqual(tiles[2], (666, 36, 1178, 360))

    def test_wide_search_roi_tile_follows_tracked_target(self):
        tile = select_horizontal_roi_tile(
            search_roi=(102, 36, 1178, 360),
            tile_width_px=512,
            tracked_bbox=(730, 120, 770, 150),
            scan_index=0,
        )

        self.assertEqual(tile, (494, 36, 1006, 360))

    def test_roi_tile_returns_search_area_when_tile_is_wider(self):
        self.assertEqual(
            select_horizontal_roi_tile(
                search_roi=(100, 30, 500, 300),
                tile_width_px=500,
                tracked_bbox=None,
                scan_index=0,
            ),
            (100, 30, 500, 300),
        )

    def test_empty_camera_poll_only_fails_after_watchdog(self):
        self.assertFalse(camera_poll_timed_out("empty", 0.49, 0.50))
        self.assertTrue(camera_poll_timed_out("empty", 0.50, 0.50))

    def test_camera_read_error_fails_immediately(self):
        self.assertTrue(camera_poll_timed_out("error", 0.01, 0.50))
        self.assertTrue(camera_poll_timed_out("shape_error", 0.01, 0.50))

    def test_normalized_upper_half_detection_roi(self):
        self.assertEqual(
            normalized_roi_to_bbox(
                (720, 1280, 3),
                0.00,
                0.0,
                1.00,
                0.50,
            ),
            (0, 0, 1280, 360),
        )

    def test_bbox_round_trip_between_frame_and_roi(self):
        roi = (0, 0, 1280, 360)
        frame_bbox = (600, 180, 630, 205)
        roi_bbox = frame_bbox_to_roi(frame_bbox, roi)
        self.assertEqual(roi_bbox, frame_bbox)
        self.assertEqual(roi_bbox_to_frame(roi_bbox, roi), frame_bbox)

    def test_bbox_outside_detection_roi_is_rejected(self):
        self.assertIsNone(
            frame_bbox_to_roi(
                (600, 400, 630, 425),
                (0, 0, 1280, 360),
            )
        )

    def test_nonnegative_median_keeps_zero_distance(self):
        median, count = robust_nonnegative_median(
            [math.nan, -1.0, 0.0, 0.4, 0.6],
            3,
        )
        self.assertAlmostEqual(median, 0.4)
        self.assertEqual(count, 3)

    def test_stopline_near_requires_current_valid_stable_measurement(self):
        arguments = dict(
            median_distance_m=0.8,
            stop_distance_m=1.0,
            current_line_detected=True,
            current_depth_accepted=True,
            line_stable=True,
            valid_distance_samples=3,
            minimum_distance_samples=3,
        )
        self.assertTrue(is_stopline_approaching(**arguments))
        for key in (
            "current_line_detected",
            "current_depth_accepted",
            "line_stable",
        ):
            invalid = dict(arguments)
            invalid[key] = False
            self.assertFalse(is_stopline_approaching(**invalid))

    def test_stopline_near_is_disabled_at_zero_threshold(self):
        self.assertFalse(
            is_stopline_approaching(
                median_distance_m=0.0,
                stop_distance_m=0.0,
                current_line_detected=True,
                current_depth_accepted=True,
                line_stable=True,
                valid_distance_samples=3,
                minimum_distance_samples=3,
            )
        )

    def test_stopline_y_threshold_uses_stable_normalized_median(self):
        arguments = dict(
            median_y_px=648.0,
            frame_height_px=720,
            stop_y_ratio=0.90,
            current_line_detected=True,
            line_stable=True,
            valid_y_samples=3,
            minimum_y_samples=3,
        )
        self.assertTrue(is_stopline_y_approaching(**arguments))

        for key in ("current_line_detected", "line_stable"):
            invalid = dict(arguments)
            invalid[key] = False
            self.assertFalse(is_stopline_y_approaching(**invalid))

        insufficient = dict(arguments)
        insufficient["valid_y_samples"] = 2
        self.assertFalse(is_stopline_y_approaching(**insufficient))

    def test_stopline_y_threshold_is_resolution_independent(self):
        common = dict(
            stop_y_ratio=0.90,
            current_line_detected=True,
            line_stable=True,
            valid_y_samples=3,
            minimum_y_samples=3,
        )
        self.assertTrue(
            is_stopline_y_approaching(
                median_y_px=648.0,
                frame_height_px=720,
                **common,
            )
        )
        self.assertTrue(
            is_stopline_y_approaching(
                median_y_px=432.0,
                frame_height_px=480,
                **common,
            )
        )

    def test_stopline_y_threshold_allows_small_below_frame_extrapolation(self):
        common = dict(
            frame_height_px=720,
            current_line_detected=True,
            line_stable=True,
            valid_y_samples=3,
            minimum_y_samples=3,
        )
        self.assertFalse(
            is_stopline_y_approaching(
                median_y_px=722.0,
                stop_y_ratio=1.005,
                **common,
            )
        )
        self.assertTrue(
            is_stopline_y_approaching(
                median_y_px=724.0,
                stop_y_ratio=1.005,
                **common,
            )
        )

    def test_stopline_y_threshold_rejects_disabled_and_invalid_values(self):
        common = dict(
            frame_height_px=720,
            current_line_detected=True,
            line_stable=True,
            valid_y_samples=3,
            minimum_y_samples=3,
        )
        self.assertFalse(
            is_stopline_y_approaching(
                median_y_px=648.0,
                stop_y_ratio=0.0,
                **common,
            )
        )
        self.assertFalse(
            is_stopline_y_approaching(
                median_y_px=647.0,
                stop_y_ratio=0.90,
                **common,
            )
        )
        self.assertFalse(
            is_stopline_y_approaching(
                median_y_px=math.nan,
                stop_y_ratio=0.90,
                **common,
            )
        )

    def test_stopline_proximity_combines_only_enabled_gates(self):
        self.assertFalse(
            combine_stopline_proximity(False, False, 0.0, 0.0)
        )
        self.assertTrue(
            combine_stopline_proximity(True, False, 3.0, 0.0)
        )
        self.assertTrue(
            combine_stopline_proximity(False, True, 0.0, 0.90)
        )
        self.assertTrue(
            combine_stopline_proximity(True, True, 3.0, 0.90)
        )
        self.assertFalse(
            combine_stopline_proximity(True, False, 3.0, 0.90)
        )
        self.assertFalse(
            combine_stopline_proximity(False, True, 3.0, 0.90)
        )

    def test_latch_stops_on_red_and_pixel_threshold(self):
        self.assertTrue(update_stop_latch(False, True, True, False, True))
        self.assertFalse(update_stop_latch(False, True, False, False, True))

    def test_red_loss_alone_never_releases_latch(self):
        self.assertTrue(update_stop_latch(True, False, False, False, True))

    def test_confirmed_green_releases_latch(self):
        self.assertFalse(update_stop_latch(True, False, False, True, True))
        self.assertTrue(update_stop_latch(True, False, False, True, False))

    def test_confirmed_red_clear_releases_latch(self):
        self.assertFalse(
            update_stop_latch(
                True,
                False,
                False,
                False,
                False,
                red_clear_active=True,
                resume_on_red_clear=True,
            )
        )

    def test_unconfirmed_red_clear_keeps_latch(self):
        self.assertTrue(
            update_stop_latch(
                True,
                False,
                False,
                False,
                False,
                red_clear_active=False,
                resume_on_red_clear=True,
            )
        )

    def test_red_wins_if_both_votes_are_active(self):
        self.assertTrue(update_stop_latch(True, True, True, True, True))

    def test_red_phase_survives_bbox_loss_until_fresh_green(self):
        phase = update_red_phase_latch(False, True, False)
        self.assertTrue(phase)
        for _ in range(50):
            phase = update_red_phase_latch(phase, False, False)
        self.assertTrue(phase)
        self.assertTrue(
            update_stop_latch(False, phase, True, False, True)
        )
        phase = update_red_phase_latch(phase, False, True)
        self.assertFalse(phase)

    def test_unconfirmed_red_never_arms_red_phase(self):
        self.assertFalse(update_red_phase_latch(False, False, False))

    def test_red_wins_over_simultaneous_green_for_phase(self):
        self.assertTrue(update_red_phase_latch(True, True, True))

    def test_anchored_green_requires_red_phase_and_saved_bbox(self):
        self.assertTrue(should_accept_anchored_green(True, True, True))
        self.assertFalse(should_accept_anchored_green(False, True, True))
        self.assertFalse(should_accept_anchored_green(True, False, True))
        self.assertFalse(should_accept_anchored_green(True, True, False))

    def test_color_ratios_are_mutually_exclusive(self):
        self.assertEqual(
            classify_color_ratios(0.10, 0.01, 0.004, 0.004),
            (True, False),
        )
        self.assertEqual(
            classify_color_ratios(0.01, 0.10, 0.004, 0.004),
            (False, True),
        )
        self.assertEqual(
            classify_color_ratios(0.001, 0.001, 0.004, 0.004),
            (False, False),
        )

    def test_only_explicit_red_or_fresh_green_advances_color_vote(self):
        self.assertFalse(should_record_color_vote(True, False, False))
        self.assertFalse(should_record_color_vote(False, False, False))
        self.assertTrue(should_record_color_vote(True, True, False))
        self.assertFalse(should_record_color_vote(False, True, False))
        self.assertTrue(
            should_record_color_vote(
                False,
                True,
                False,
                red_fresh_seeded=True,
            )
        )
        self.assertTrue(should_record_color_vote(True, False, True))
        self.assertFalse(should_record_color_vote(False, False, True))

    def test_bbox_iou(self):
        self.assertAlmostEqual(
            bbox_iou((0, 0, 10, 10), (5, 5, 15, 15)),
            25.0 / 175.0,
        )
        self.assertEqual(
            bbox_iou((0, 0, 10, 10), (20, 20, 30, 30)),
            0.0,
        )

    def test_tracking_keeps_nearby_low_confidence_bbox(self):
        previous_bbox = (100, 100, 120, 130)
        selected = select_tracking_candidate(
            [
                ((102, 101, 122, 131), 0.12),
                ((300, 100, 330, 140), 0.90),
            ],
            previous_bbox,
            minimum_iou=0.10,
            maximum_center_shift_ratio=0.50,
            minimum_size_similarity=0.50,
        )
        self.assertEqual(selected, ((102, 101, 122, 131), 0.12))

    def test_tracking_rejects_unrelated_bbox(self):
        selected = select_tracking_candidate(
            [((300, 100, 330, 140), 0.90)],
            (100, 100, 120, 130),
            minimum_iou=0.10,
            maximum_center_shift_ratio=0.50,
            minimum_size_similarity=0.50,
        )
        self.assertIsNone(selected)

    def test_tracking_rejects_adjacent_and_oversized_bbox(self):
        previous_bbox = (100, 100, 120, 130)
        selected = select_tracking_candidate(
            [
                ((120, 100, 140, 130), 0.80),
                ((70, 55, 150, 175), 0.90),
            ],
            previous_bbox,
            minimum_iou=0.10,
            maximum_center_shift_ratio=0.50,
            minimum_size_similarity=0.50,
        )
        self.assertIsNone(selected)

    def test_red_clear_requires_four_real_bbox_observations(self):
        self.assertTrue(
            is_red_clear_confirmed(
                [0, 0, 0, 0, 0],
                [1, 1, 0, 1, 1],
                window_size=5,
                minimum_bbox_observations=4,
            )
        )
        self.assertFalse(
            is_red_clear_confirmed(
                [0, 0, 0, 0, 0],
                [1, 1, 0, 0, 0],
                window_size=5,
                minimum_bbox_observations=4,
            )
        )

    def test_five_missed_frames_never_confirm_red_clear(self):
        self.assertFalse(
            is_red_clear_confirmed(
                [0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0],
                window_size=5,
                minimum_bbox_observations=4,
            )
        )


if __name__ == "__main__":
    unittest.main()
