"""ROS-independent dynamic obstacle tracking for :mod:`stack_estop`.

This is an in-package adaptation of the validated ``motion_core.py`` from
``ydlidar_ws/sudden_obstacle_detector``.  Unlike the prototype, this core
accepts points that the caller has already transformed into ``base_link``.
It never creates ROS entities and never publishes a final E-Stop request.
"""

from collections import deque
import math

import numpy as np


DYNAMIC_DEFAULTS = {
    "background_min_x_m": 0.20,
    "background_max_x_m": 5.00,
    "background_min_abs_y_m": 0.45,
    "roi_min_x_m": 0.15,
    "roi_max_x_m": 1.50,
    "tracking_max_x_m": 3.00,
    "roi_half_width_m": 0.90,
    "corridor_half_width_m": 0.30,
    "cluster_min_points": 4,
    "cluster_max_index_gap": 1,
    "cluster_gap_m": 0.12,
    "icp_max_iterations": 15,
    "icp_correspondence_distance_m": 0.25,
    "icp_trim_ratio": 0.75,
    "icp_convergence_translation_m": 0.002,
    "icp_convergence_rotation_rad": 0.002,
    "icp_max_points": 180,
    "icp_min_points": 30,
    "icp_min_correspondences": 20,
    "icp_min_inlier_ratio": 0.50,
    "icp_max_rmse_m": 0.10,
    "icp_max_translation_m": 0.50,
    "icp_max_rotation_deg": 15.0,
    "association_distance_m": 0.30,
    "association_max_lateral_span_diff_m": 0.30,
    "association_point_ratio_min": 0.33,
    "association_point_ratio_max": 3.0,
    "track_max_missing_frames": 2,
    "dormant_reconnect_max_frames": 2,
    "dormant_reconnect_max_time_sec": 0.35,
    "dormant_reconnect_distance_m": 0.65,
    "dormant_reconnect_max_delta_x_m": 0.35,
    "dormant_reconnect_max_lateral_span_diff_m": 0.25,
    "dormant_reconnect_point_ratio_min": 0.50,
    "dormant_reconnect_point_ratio_max": 2.50,
    "dormant_reconnect_min_lateral_motion_m": 0.05,
    "dormant_reconnect_ambiguity_margin_m": 0.08,
    "velocity_history_frames": 4,
    "corridor_approach_vy_threshold_mps": 0.20,
    "prediction_horizon_sec": 0.50,
    "dynamic_confirm_frames": 3,
    "dynamic_release_frames": 2,
    "minimum_track_age_frames": 3,
    "dynamic_residual_vy_threshold_mps": 0.12,
    "dynamic_frame_max_yaw_deg": 2.0,
    "dynamic_cumulative_lateral_displacement_m": 0.04,
    "motion_window_frames": 4,
    "track_birth_far_margin_m": 0.25,
    "track_birth_side_margin_m": 0.10,
    "inside_birth_confirm_frames": 2,
    "side_observe_frames": 2,
    "corridor_entry_margin_m": 0.05,
    "hazard_clear_frames": 5,
    "dynamic_stop_distance_m": 1.00,
}


def _transform_points(points, rotation, translation):
    if len(points) == 0:
        return np.empty((0, 2), dtype=float)
    return np.asarray(points, dtype=float) @ rotation.T + translation


def _transform_point(point, rotation, translation):
    return rotation @ np.asarray(point, dtype=float) + translation


def estimate_trimmed_icp(previous, current, params):
    """Estimate the previous-to-current SE(2) transform."""
    previous = np.asarray(previous, dtype=float)
    current = np.asarray(current, dtype=float)
    maximum = int(params["icp_max_points"])
    if len(previous) > maximum:
        previous = previous[np.linspace(0, len(previous) - 1, maximum, dtype=int)]
    if len(current) > maximum:
        current = current[np.linspace(0, len(current) - 1, maximum, dtype=int)]
    result = {
        "valid": False,
        "rotation": np.eye(2),
        "translation": np.zeros(2),
        "correspondence_count": 0,
        "inlier_ratio": 0.0,
        "rmse_m": None,
        "delta_x": 0.0,
        "delta_y": 0.0,
        "delta_yaw": 0.0,
    }
    if len(previous) < 2 or len(current) < 2:
        return result
    rotation = np.eye(2)
    translation = np.zeros(2)
    raw_count = used_count = 0
    rmse = None
    try:
        for _ in range(int(params["icp_max_iterations"])):
            moved = _transform_points(previous, rotation, translation)
            distances_sq = np.sum(
                (moved[:, None, :] - current[None, :, :]) ** 2, axis=2)
            nearest_indices = np.argmin(distances_sq, axis=1)
            nearest_distances = np.sqrt(
                distances_sq[np.arange(len(moved)), nearest_indices])
            accepted = np.flatnonzero(
                nearest_distances <= params["icp_correspondence_distance_m"])
            raw_count = len(accepted)
            if raw_count < 2:
                break
            keep = max(2, int(math.floor(raw_count * params["icp_trim_ratio"])))
            accepted = accepted[np.argsort(nearest_distances[accepted])[:keep]]
            source = moved[accepted]
            target = current[nearest_indices[accepted]]
            source_center = np.mean(source, axis=0)
            target_center = np.mean(target, axis=0)
            covariance = (source - source_center).T @ (target - target_center)
            u_matrix, _, vt_matrix = np.linalg.svd(covariance)
            increment_rotation = vt_matrix.T @ u_matrix.T
            if np.linalg.det(increment_rotation) < 0:
                vt_matrix[-1, :] *= -1
                increment_rotation = vt_matrix.T @ u_matrix.T
            increment_translation = (
                target_center - increment_rotation @ source_center)
            rotation = increment_rotation @ rotation
            translation = increment_rotation @ translation + increment_translation
            aligned = source @ increment_rotation.T + increment_translation
            errors = np.linalg.norm(aligned - target, axis=1)
            rmse = float(np.sqrt(np.mean(errors ** 2)))
            used_count = len(accepted)
            step_yaw = math.atan2(
                increment_rotation[1, 0], increment_rotation[0, 0])
            if (
                np.linalg.norm(increment_translation)
                <= params["icp_convergence_translation_m"]
                and abs(step_yaw) <= params["icp_convergence_rotation_rad"]
            ):
                break
    except (np.linalg.LinAlgError, ValueError, FloatingPointError):
        return result
    yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    inlier_ratio = raw_count / max(1, min(len(previous), len(current)))
    result.update({
        "rotation": rotation,
        "translation": translation,
        "correspondence_count": used_count,
        "inlier_ratio": inlier_ratio,
        "rmse_m": rmse,
        "delta_x": float(translation[0]),
        "delta_y": float(translation[1]),
        "delta_yaw": yaw,
    })
    result["valid"] = bool(
        len(previous) >= params["icp_min_points"]
        and len(current) >= params["icp_min_points"]
        and used_count >= params["icp_min_correspondences"]
        and inlier_ratio >= params["icp_min_inlier_ratio"]
        and rmse is not None and rmse <= params["icp_max_rmse_m"]
        and np.linalg.norm(translation) <= params["icp_max_translation_m"]
        and abs(math.degrees(yaw)) <= params["icp_max_rotation_deg"])
    return result


def _features(cluster):
    xy = np.asarray([[point[1], point[2]] for point in cluster], dtype=float)
    return {
        "centroid": np.mean(xy, axis=0),
        "centroid_x": float(np.mean(xy[:, 0])),
        "centroid_y": float(np.mean(xy[:, 1])),
        "nearest_x": float(np.min(xy[:, 0])),
        "min_y": float(np.min(xy[:, 1])),
        "max_y": float(np.max(xy[:, 1])),
        "lateral_span": float(np.ptp(xy[:, 1])),
        "point_count": len(cluster),
    }


def cluster_points(points, params):
    clusters = []
    for point in points:
        if not clusters:
            clusters.append([point])
            continue
        previous = clusters[-1][-1]
        index_close = (
            point[0] - previous[0]
            <= int(params["cluster_max_index_gap"]) + 1)
        spatial_close = math.hypot(
            point[1] - previous[1], point[2] - previous[2]
        ) <= params["cluster_gap_m"]
        if index_close and spatial_close:
            clusters[-1].append(point)
        else:
            clusters.append([point])
    return [
        _features(cluster) for cluster in clusters
        if len(cluster) >= int(params["cluster_min_points"])
    ]


class DynamicMotionCore:
    """Track motion and return dynamic hazard evidence in ``base_link``."""

    def __init__(self, params=None):
        self.params = dict(DYNAMIC_DEFAULTS)
        if params:
            self.params.update(params)
        self.previous_background = None
        self.previous_timestamp = None
        self.tracks = {}
        self.next_track_id = 1
        self.hazard_latched = False
        self.dynamic_estop_latched = False
        self.hazard_clear_count = 0
        self.selected_hazard_track_id = None
        self.selected_hazard_x = None
        self.hazard_registration_type = None
        self.association_diagnostics = {}

    def _update_motion_evidence(
        self, track, frame_valid, residual_vy, centroid_y,
    ):
        track["filtered_residual_vy"] = float(residual_vy)
        history = track["motion_lateral_history"]
        if frame_valid:
            history.append(float(centroid_y))
        else:
            history.clear()
        displacement = (
            abs(history[-1] - history[0]) if len(history) >= 2 else 0.0)
        sign = 1 if residual_vy > 0.0 else -1 if residual_vy < 0.0 else 0
        consistent = track["previous_residual_vy_sign"] in (0, sign)
        evidence = bool(
            frame_valid
            and abs(residual_vy)
            >= self.params["dynamic_residual_vy_threshold_mps"]
            and displacement
            >= self.params["dynamic_cumulative_lateral_displacement_m"]
            and sign != 0 and consistent)
        if evidence:
            track["motion_evidence_count"] += 1
            track["motion_release_count"] = 0
            track["previous_residual_vy_sign"] = sign
            if track["motion_evidence_count"] >= self.params["dynamic_confirm_frames"]:
                track["dynamic_confirmed"] = True
        else:
            track["motion_evidence_count"] = 0
            if track["dynamic_confirmed"]:
                track["motion_release_count"] += 1
                if track["motion_release_count"] >= self.params["dynamic_release_frames"]:
                    track["dynamic_confirmed"] = False
                    track["motion_release_count"] = 0
            if not frame_valid:
                track["previous_residual_vy_sign"] = 0
            elif abs(residual_vy) >= self.params["dynamic_residual_vy_threshold_mps"]:
                track["previous_residual_vy_sign"] = sign
        track["cumulative_lateral_displacement_m"] = displacement

    def _new_track(self, cluster, timestamp):
        far = cluster["centroid_x"] >= (
            self.params["roi_max_x_m"] - self.params["track_birth_far_margin_m"])
        side = abs(cluster["centroid_y"]) >= (
            self.params["roi_half_width_m"]
            - self.params["track_birth_side_margin_m"])
        inside = abs(cluster["centroid_y"]) <= self.params["corridor_half_width_m"]
        birth_type = "FAR_BOUNDARY" if far else (
            "SIDE_BOUNDARY" if side else "INSIDE" if inside else "FAR_BOUNDARY")
        outside = abs(cluster["centroid_y"]) > (
            self.params["corridor_half_width_m"]
            + self.params["corridor_entry_margin_m"])
        track = {
            "track_id": self.next_track_id,
            **cluster,
            "age_frames": 1,
            "missing_frames": 0,
            "last_seen_time": timestamp,
            "velocity_history": deque(
                maxlen=int(self.params["velocity_history_frames"])),
            "residual_vx_mps": 0.0,
            "residual_vy_mps": 0.0,
            "moving_count": 0,
            "birth_type": birth_type,
            "first_seen_x": cluster["centroid_x"],
            "inside_corridor_frames": 1 if inside else 0,
            "outside_corridor_frames": 1 if outside else 0,
            "was_outside_corridor": False,
            "last_confirmed_outside_y": cluster["centroid_y"] if outside else None,
            "confirmed_side_entry": False,
            "valid_inside_appearance": False,
            "side_entry_event": False,
            "inside_appearance_event": False,
            "registration_event_emitted": False,
            "motion_evidence_count": 0,
            "motion_release_count": 0,
            "filtered_residual_vy": 0.0,
            "previous_residual_vy_sign": 0,
            "dynamic_confirmed": False,
            "motion_lateral_history": deque(
                maxlen=int(self.params["motion_window_frames"])),
            "cumulative_lateral_displacement_m": 0.0,
            # This flag is set only by consistent motion toward the corridor.
            # It permits a short, strictly gated dormant re-identification;
            # static outside tracks never become eligible.
            "dormant_reconnect_eligible": False,
        }
        self.next_track_id += 1
        return track

    def _associate(self, clusters, match, timestamp):
        rotation = match["rotation"] if match["valid"] else np.eye(2)
        translation = match["translation"] if match["valid"] else np.zeros(2)
        dt = timestamp - self.previous_timestamp if self.previous_timestamp is not None else None
        velocity_valid = match["valid"] and dt is not None and 0.0 < dt <= 0.5
        for track in self.tracks.values():
            track["side_entry_event"] = False
            track["inside_appearance_event"] = False
        predictions = {
            track_id: _transform_point(track["centroid"], rotation, translation)
            for track_id, track in self.tracks.items()
        }
        self.association_diagnostics = {
            "association_reason": "NO_ASSOCIATION_CANDIDATE",
            "reconnected_previous_track_id": None,
            "association_predicted_x": None,
            "association_predicted_y": None,
            "association_distance_m": None,
            "dormant_frame_count": 0,
            "association_ambiguous": False,
            "dormant_reconnect_rejection_reason": None,
        }
        candidates = []
        for track_id, track in self.tracks.items():
            for cluster_index, cluster in enumerate(clusters):
                distance = float(np.linalg.norm(
                    cluster["centroid"] - predictions[track_id]))
                span_diff = abs(cluster["lateral_span"] - track["lateral_span"])
                ratio = cluster["point_count"] / max(1, track["point_count"])
                if (
                    distance <= self.params["association_distance_m"]
                    and span_diff
                    <= self.params["association_max_lateral_span_diff_m"]
                    and self.params["association_point_ratio_min"]
                    <= ratio <= self.params["association_point_ratio_max"]
                ):
                    candidates.append((distance, track_id, cluster_index))
        used_tracks, used_clusters, associations = set(), set(), []
        for distance, track_id, cluster_index in sorted(candidates):
            if track_id not in used_tracks and cluster_index not in used_clusters:
                used_tracks.add(track_id)
                used_clusters.add(cluster_index)
                associations.append((
                    track_id, cluster_index, False,
                    predictions[track_id], distance,
                    self.tracks[track_id]["missing_frames"],
                ))

        # A separate, strict re-identification pass handles only a recently
        # lost outside track which had consistent motion toward the corridor.
        # The normal 0.30 m association gate above is deliberately unchanged.
        dormant_candidates = []
        best_dormant_rejection = (float("inf"), None)
        corridor = self.params["corridor_half_width_m"]
        for track_id, track in self.tracks.items():
            if track_id in used_tracks:
                continue
            dormant_frames = int(track["missing_frames"])
            if not 1 <= dormant_frames <= int(
                    self.params["dormant_reconnect_max_frames"]):
                continue
            if not (
                track["dormant_reconnect_eligible"]
                and track["was_outside_corridor"]
                and track["previous_residual_vy_sign"] != 0
            ):
                best_dormant_rejection = (
                    0.0, "DORMANT_NOT_MOTION_ELIGIBLE")
                continue
            elapsed = float(timestamp - track["last_seen_time"])
            if not 0.0 < elapsed <= self.params["dormant_reconnect_max_time_sec"]:
                best_dormant_rejection = (0.0, "DORMANT_TIME_GAP_HIGH")
                continue
            velocity_horizon = min(
                elapsed, self.params["prediction_horizon_sec"])
            predicted = predictions[track_id] + np.asarray((
                track["residual_vx_mps"] * velocity_horizon,
                track["residual_vy_mps"] * velocity_horizon,
            ))
            for cluster_index, cluster in enumerate(clusters):
                if cluster_index in used_clusters:
                    continue
                delta = cluster["centroid"] - track["centroid"]
                direction_sign = 1 if delta[1] > 0.0 else -1 if delta[1] < 0.0 else 0
                outside_y = track["last_confirmed_outside_y"]
                direction_toward = bool(
                    outside_y is not None
                    and outside_y * delta[1] < 0.0
                    and direction_sign == track["previous_residual_vy_sign"])
                distance = float(np.linalg.norm(cluster["centroid"] - predicted))
                span_diff = abs(
                    cluster["lateral_span"] - track["lateral_span"])
                ratio = cluster["point_count"] / max(1, track["point_count"])
                if not match["valid"]:
                    rejection = "DORMANT_ICP_INVALID"
                elif abs(cluster["centroid_y"]) > corridor:
                    rejection = "DORMANT_NOT_INSIDE_CORRIDOR"
                elif not direction_toward:
                    rejection = "DORMANT_DIRECTION_MISMATCH"
                elif abs(delta[1]) < self.params[
                        "dormant_reconnect_min_lateral_motion_m"]:
                    rejection = "DORMANT_LATERAL_MOTION_LOW"
                elif abs(
                    cluster["centroid_x"] - predictions[track_id][0]
                ) > self.params["dormant_reconnect_max_delta_x_m"]:
                    rejection = "DORMANT_LONGITUDINAL_DELTA_HIGH"
                elif distance > self.params["dormant_reconnect_distance_m"]:
                    rejection = "DORMANT_PREDICTION_DISTANCE_HIGH"
                elif span_diff > self.params[
                        "dormant_reconnect_max_lateral_span_diff_m"]:
                    rejection = "DORMANT_SHAPE_SPAN_MISMATCH"
                elif not (
                    self.params["dormant_reconnect_point_ratio_min"]
                    <= ratio <= self.params["dormant_reconnect_point_ratio_max"]
                ):
                    rejection = "DORMANT_POINT_RATIO_MISMATCH"
                else:
                    rejection = None
                if rejection is None:
                    dormant_candidates.append((
                        distance, track_id, cluster_index, predicted,
                        dormant_frames))
                elif distance < best_dormant_rejection[0]:
                    best_dormant_rejection = (distance, rejection)

        self.association_diagnostics[
            "dormant_reconnect_rejection_reason"] = best_dormant_rejection[1]

        by_cluster = {}
        by_track = {}
        for item in dormant_candidates:
            by_cluster.setdefault(item[2], []).append(item)
            by_track.setdefault(item[1], []).append(item)
        ambiguous_tracks = set()
        for track_id, options in by_track.items():
            options.sort(key=lambda item: item[0])
            if (
                len(options) > 1
                and options[1][0] - options[0][0]
                <= self.params["dormant_reconnect_ambiguity_margin_m"]
            ):
                ambiguous_tracks.add(track_id)
                self.association_diagnostics.update({
                    "association_reason": "DORMANT_MATCH_AMBIGUOUS",
                    "association_ambiguous": True,
                    "dormant_reconnect_rejection_reason":
                        "DORMANT_MATCH_AMBIGUOUS",
                })
        for cluster_index, options in sorted(by_cluster.items()):
            options = [
                item for item in options if item[1] not in ambiguous_tracks]
            if not options:
                continue
            options.sort(key=lambda item: item[0])
            if (
                len(options) > 1
                and options[1][0] - options[0][0]
                <= self.params["dormant_reconnect_ambiguity_margin_m"]
            ):
                self.association_diagnostics.update({
                    "association_reason": "DORMANT_MATCH_AMBIGUOUS",
                    "association_ambiguous": True,
                    "dormant_reconnect_rejection_reason":
                        "DORMANT_MATCH_AMBIGUOUS",
                })
                continue
            distance, track_id, _, predicted, dormant_frames = options[0]
            if track_id in used_tracks or cluster_index in used_clusters:
                continue
            used_tracks.add(track_id)
            used_clusters.add(cluster_index)
            associations.append((
                track_id, cluster_index, True, predicted, distance,
                dormant_frames))

        for (
            track_id, cluster_index, reconnected, predicted,
            association_distance, dormant_frames,
        ) in associations:
            track = self.tracks[track_id]
            cluster = clusters[cluster_index]
            residual = cluster["centroid"] - predicted
            if velocity_valid:
                track["velocity_history"].append(
                    (float(residual[0] / dt), float(residual[1] / dt)))
                velocities = np.asarray(track["velocity_history"], dtype=float)
                residual_vx = float(np.median(velocities[:, 0]))
                residual_vy = float(np.median(velocities[:, 1]))
            else:
                track["velocity_history"].clear()
                residual_vx = residual_vy = 0.0
            corridor = self.params["corridor_half_width_m"]
            moving = abs(residual_vy) >= self.params["corridor_approach_vy_threshold_mps"]
            track["moving_count"] = track["moving_count"] + 1 if moving and velocity_valid else 0
            track.update({
                **cluster,
                "age_frames": track["age_frames"] + 1,
                "missing_frames": 0,
                "last_seen_time": timestamp,
                "residual_vx_mps": residual_vx,
                "residual_vy_mps": residual_vy,
            })
            entry_boundary = corridor + self.params["corridor_entry_margin_m"]
            current_inside = abs(cluster["centroid_y"]) <= corridor
            if abs(cluster["centroid_y"]) > entry_boundary:
                track["outside_corridor_frames"] += 1
                track["last_confirmed_outside_y"] = cluster["centroid_y"]
                if track["outside_corridor_frames"] >= self.params["side_observe_frames"]:
                    track["was_outside_corridor"] = True
            if current_inside:
                track["inside_corridor_frames"] += 1
            elif track["birth_type"] == "INSIDE":
                track["inside_corridor_frames"] = 0
            frame_valid = bool(
                velocity_valid
                and abs(math.degrees(match["delta_yaw"]))
                <= self.params["dynamic_frame_max_yaw_deg"]
                and track["age_frames"] >= self.params["minimum_track_age_frames"]
                and cluster["point_count"] >= self.params["cluster_min_points"]
                and cluster["nearest_x"] <= self.params["tracking_max_x_m"])
            self._update_motion_evidence(
                track, frame_valid, residual_vy, cluster["centroid_y"])
            if (
                track["was_outside_corridor"]
                and track["motion_evidence_count"] > 0
                and track["last_confirmed_outside_y"] is not None
                and track["last_confirmed_outside_y"]
                * track["previous_residual_vy_sign"] < 0.0
            ):
                track["dormant_reconnect_eligible"] = True
            outside_y = track["last_confirmed_outside_y"]
            displacement = (
                abs(outside_y - cluster["centroid_y"])
                if outside_y is not None else 0.0)
            direction_toward = bool(
                outside_y is not None
                and outside_y * track["filtered_residual_vy"] < 0.0
                and track["previous_residual_vy_sign"]
                == (-1 if outside_y > 0.0 else 1))
            side_event = bool(
                track["was_outside_corridor"] and current_inside
                and displacement
                >= self.params["dynamic_cumulative_lateral_displacement_m"]
                and direction_toward and frame_valid
                and track["dynamic_confirmed"]
                and not track["registration_event_emitted"])
            inside_event = bool(
                track["birth_type"] == "INSIDE"
                and current_inside
                and track["inside_corridor_frames"]
                >= self.params["inside_birth_confirm_frames"]
                and track["first_seen_x"]
                < self.params["roi_max_x_m"] - self.params["track_birth_far_margin_m"]
                and track["dynamic_confirmed"]
                and not track["registration_event_emitted"])
            if side_event:
                track["confirmed_side_entry"] = True
                track["side_entry_event"] = True
            if inside_event:
                track["valid_inside_appearance"] = True
                track["inside_appearance_event"] = True
            if side_event or inside_event:
                track["registration_event_emitted"] = True
                self.hazard_latched = True
                self.selected_hazard_track_id = track_id
                self.selected_hazard_x = track["nearest_x"]
                self.hazard_registration_type = (
                    "SIDE_ENTRY" if side_event else "INSIDE_APPEARANCE")
            # Do not hide a rejected ambiguous dormant match merely because
            # an unrelated ordinary track was associated in the same scan.
            if reconnected or not self.association_diagnostics[
                    "association_ambiguous"]:
                self.association_diagnostics.update({
                    "association_reason": (
                        "DORMANT_RECONNECTED" if reconnected
                        else "REGULAR_ASSOCIATED"),
                    "reconnected_previous_track_id": (
                        track_id if reconnected else None),
                    "association_predicted_x": float(predicted[0]),
                    "association_predicted_y": float(predicted[1]),
                    "association_distance_m": float(association_distance),
                    "dormant_frame_count": dormant_frames,
                    "association_ambiguous": False,
                    "dormant_reconnect_rejection_reason": (
                        None if reconnected else self.association_diagnostics[
                            "dormant_reconnect_rejection_reason"]),
                })

        for track_id in list(self.tracks):
            if track_id not in used_tracks:
                track = self.tracks[track_id]
                track["missing_frames"] += 1
                if not (
                    track["dormant_reconnect_eligible"]
                    and track["missing_frames"]
                    <= self.params["dormant_reconnect_max_frames"]
                ):
                    self._update_motion_evidence(
                        track, False, track["filtered_residual_vy"],
                        track["centroid_y"])
                # A registered hazard keeps its identity and last position
                # through association dropouts.  It is discarded only after
                # the existing empty-corridor clear confirmation releases the
                # hazard latch; ordinary tracks retain the original timeout.
                if (
                    track["missing_frames"]
                    > self.params["track_max_missing_frames"]
                    and not (
                        self.hazard_latched
                        and track_id == self.selected_hazard_track_id
                    )
                ):
                    del self.tracks[track_id]
        for cluster_index, cluster in enumerate(clusters):
            if cluster_index not in used_clusters:
                track = self._new_track(cluster, timestamp)
                self.tracks[track["track_id"]] = track

    def process(self, base_points, timestamp, scan_valid=True):
        """Process ``(index, x, y, range)`` points in ``base_link``."""
        background = np.asarray([
            (point[1], point[2]) for point in base_points
            if self.params["background_min_x_m"] <= point[1]
            <= self.params["background_max_x_m"]
            and abs(point[2]) >= self.params["background_min_abs_y_m"]
        ], dtype=float)
        if background.size == 0:
            background = np.empty((0, 2), dtype=float)
        watch = [
            point for point in base_points
            if self.params["roi_min_x_m"] <= point[1]
            <= self.params["tracking_max_x_m"]
            and abs(point[2]) <= self.params["roi_half_width_m"]
        ]
        match = estimate_trimmed_icp(
            [] if self.previous_background is None else self.previous_background,
            background, self.params)
        clusters = cluster_points(watch, self.params)
        self._associate(clusters, match, float(timestamp))

        selected = self.tracks.get(self.selected_hazard_track_id)
        if selected is not None and selected["missing_frames"] == 0:
            self.selected_hazard_x = selected["nearest_x"]
        corridor_occupied = any(
            abs(cluster["centroid_y"]) <= self.params["corridor_half_width_m"]
            for cluster in clusters)
        if self.hazard_latched:
            if scan_valid and not corridor_occupied:
                self.hazard_clear_count += 1
                if self.hazard_clear_count >= self.params["hazard_clear_frames"]:
                    self.hazard_latched = False
                    self.dynamic_estop_latched = False
                    self.hazard_clear_count = 0
                    self.selected_hazard_track_id = None
                    self.selected_hazard_x = None
                    self.hazard_registration_type = None
            else:
                self.hazard_clear_count = 0
        if (
            self.hazard_latched
            and self.selected_hazard_x is not None
            and 0.0 < self.selected_hazard_x
            <= self.params["dynamic_stop_distance_m"]
        ):
            self.dynamic_estop_latched = True
        dynamic_estop = bool(self.dynamic_estop_latched)
        dynamic_count = sum(
            track["dynamic_confirmed"] for track in self.tracks.values())
        selected = self.tracks.get(self.selected_hazard_track_id)
        visible_tracks = [
            track for track in self.tracks.values()
            if track["missing_frames"] == 0
        ]
        candidate = max(
            visible_tracks,
            key=lambda track: (
                bool(track["dynamic_confirmed"]),
                track["motion_evidence_count"],
                track["cumulative_lateral_displacement_m"],
                -track["nearest_x"],
            ),
            default=None,
        )
        if not match["valid"]:
            rejection_reason = "ICP_INVALID"
        elif not clusters:
            rejection_reason = "NO_DYNAMIC_CLUSTER"
        elif candidate is None:
            rejection_reason = "TRACK_NOT_ASSOCIATED"
        elif candidate["age_frames"] < self.params["minimum_track_age_frames"]:
            rejection_reason = "TRACK_AGE_LOW"
        elif (
            abs(candidate["residual_vy_mps"])
            < self.params["dynamic_residual_vy_threshold_mps"]
        ):
            rejection_reason = "RESIDUAL_VY_LOW"
        elif (
            candidate["cumulative_lateral_displacement_m"]
            < self.params["dynamic_cumulative_lateral_displacement_m"]
        ):
            rejection_reason = "LATERAL_DISPLACEMENT_LOW"
        elif not candidate["dynamic_confirmed"]:
            rejection_reason = "CONFIRM_FRAMES_LOW"
        elif not candidate["was_outside_corridor"]:
            rejection_reason = "NO_OUTSIDE_HISTORY"
        elif (
            abs(candidate["centroid_y"])
            > self.params["corridor_half_width_m"]
        ):
            rejection_reason = "NO_CORRIDOR_CROSSING"
        elif not candidate["registration_event_emitted"]:
            rejection_reason = "ENTRY_DIRECTION_NOT_CONFIRMED"
        else:
            rejection_reason = "HAZARD_REGISTERED"
        hazard_stopped = bool(
            selected is not None
            and selected["missing_frames"] == 0
            and abs(selected["residual_vy_mps"])
            < self.params["dynamic_residual_vy_threshold_mps"]
        )
        if not self.hazard_latched:
            latch_reason = "NOT_LATCHED"
        elif selected is None or selected["missing_frames"] > 0:
            latch_reason = "HAZARD_TRACK_MISSING_HOLD"
        elif hazard_stopped:
            latch_reason = "REGISTERED_HAZARD_STOPPED"
        else:
            latch_reason = "REGISTERED_HAZARD_TRACKED"
        result = {
            "icp_valid": match["valid"],
            "icp_translation_x": match["delta_x"],
            "icp_translation_y": match["delta_y"],
            "icp_rotation_rad": match["delta_yaw"],
            "icp_rmse_m": match["rmse_m"],
            "dynamic_track_count": dynamic_count,
            "selected_dynamic_track_id": self.selected_hazard_track_id,
            "dynamic_x": self.selected_hazard_x,
            "dynamic_y": selected["centroid_y"] if selected else None,
            "residual_vx": selected["residual_vx_mps"] if selected else None,
            "residual_vy": selected["residual_vy_mps"] if selected else None,
            "cumulative_lateral_displacement": (
                selected["cumulative_lateral_displacement_m"] if selected else None),
            "dynamic_confirm_count": (
                selected["motion_evidence_count"] if selected else 0),
            "side_entry_event": any(
                track["side_entry_event"] for track in self.tracks.values()),
            "inside_appearance_event": any(
                track["inside_appearance_event"] for track in self.tracks.values()),
            "hazard_latched": self.hazard_latched,
            "dynamic_estop": dynamic_estop,
            "dynamic_stop_distance_m": self.params["dynamic_stop_distance_m"],
            "dynamic_tracking_max_distance_m": self.params["tracking_max_x_m"],
            "candidate_track_id": candidate["track_id"] if candidate else None,
            "candidate_x": candidate["nearest_x"] if candidate else None,
            "candidate_y": candidate["centroid_y"] if candidate else None,
            "candidate_outside_history": (
                candidate["was_outside_corridor"] if candidate else False),
            "candidate_side_entry_confirmed": (
                candidate["confirmed_side_entry"] if candidate else False),
            "hazard_track_id": self.selected_hazard_track_id,
            "hazard_stopped": hazard_stopped,
            "hazard_registration_type": self.hazard_registration_type,
            "latch_reason": latch_reason,
            "dynamic_rejection_reason": rejection_reason,
            "hazard_clear_count": self.hazard_clear_count,
            **self.association_diagnostics,
            "tracks": list(self.tracks.values()),
        }
        self.previous_background = background
        self.previous_timestamp = float(timestamp)
        return result

    def hold_after_error(self):
        """Return the last safe-to-use dynamic level without new confirmation."""
        return bool(self.dynamic_estop_latched)
