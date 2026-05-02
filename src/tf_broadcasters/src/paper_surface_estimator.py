#!/usr/bin/env python3

from __future__ import print_function

import json
import os

import numpy as np
import rospy
from assistive_msgs.msg import TagPose
from assistive_msgs.msg import TagPoseArray


class PaperSurfaceEstimator(object):
    def __init__(self):
        rospy.init_node("paper_surface_estimator", anonymous=False)

        self.layout_json_path = os.path.expanduser(rospy.get_param("~layout_json_path", ""))
        if not self.layout_json_path:
            rospy.logfatal("~layout_json_path must be set.")
            raise RuntimeError("Missing layout_json_path")

        self.input_pose_topic_name = rospy.get_param("~input_pose_topic_name", "paper_tag_poses")
        self.output_pose_topic_name = rospy.get_param("~output_pose_topic_name", "paper_surface_state")
        self.measurement_weight = float(rospy.get_param("~measurement_weight", 1.0))
        self.temporal_weight = float(rospy.get_param("~temporal_weight", 0.3))
        self.edge_weight = float(rospy.get_param("~edge_weight", 0.35))
        self.smoothness_weight = float(rospy.get_param("~smoothness_weight", 0.15))
        self.prediction_variance_growth = float(rospy.get_param("~prediction_variance_growth", 1.5))
        self.missing_position_variance_step = float(rospy.get_param("~missing_position_variance_step", 2.5e-4))
        self.missing_orientation_variance_step = float(rospy.get_param("~missing_orientation_variance_step", 5.0e-3))

        layout_data = self._load_layout(self.layout_json_path)
        self.layout_dict_name = layout_data.get("grid", {}).get("dict", "DICT_4X4_50")
        self.id_to_center, self.id_to_rowcol, self.rowcol_to_id = self._read_layout(layout_data)
        self.layout_ids = sorted(self.id_to_center.keys())
        self.id_to_index = {marker_id: idx for idx, marker_id in enumerate(self.layout_ids)}
        self.neighbors = self._build_neighbors()
        self.edges = self._build_edges()

        self.prev_points = None
        self.prev_normals = None
        self.prev_covariances = {}
        self.prev_frame_id = None

        self.pose_pub = rospy.Publisher(self.output_pose_topic_name, TagPoseArray, queue_size=1)
        self.pose_sub = rospy.Subscriber(self.input_pose_topic_name, TagPoseArray, self._pose_cb, queue_size=1)

        rospy.loginfo("paper_surface_estimator initialized.")
        rospy.loginfo("Publishing surface state on: %s", self.output_pose_topic_name)

    def _load_layout(self, path):
        if not os.path.exists(path):
            raise IOError("Layout json not found: {}".format(path))
        with open(path, "r") as f:
            return json.load(f)

    def _read_layout(self, layout_data):
        id_to_center = {}
        id_to_rowcol = {}
        rowcol_to_id = {}
        for marker in layout_data.get("markers", []):
            marker_id = int(marker["id"])
            row = int(marker["row"])
            col = int(marker["col"])
            corners = np.asarray(marker.get("corners_m", []), dtype=np.float64)
            if corners.shape != (4, 3):
                raise ValueError("Marker {} is missing corners_m".format(marker_id))
            center = np.mean(corners, axis=0)
            id_to_center[marker_id] = center
            id_to_rowcol[marker_id] = (row, col)
            rowcol_to_id[(row, col)] = marker_id
        if not id_to_center:
            raise ValueError("Layout has no markers")
        return id_to_center, id_to_rowcol, rowcol_to_id

    def _build_neighbors(self):
        neighbors = {marker_id: [] for marker_id in self.layout_ids}
        for marker_id, (row, col) in self.id_to_rowcol.items():
            candidate_rcs = [
                (row - 1, col),
                (row + 1, col),
                (row, col - 1),
                (row, col + 1),
            ]
            for candidate_rc in candidate_rcs:
                neighbor_id = self.rowcol_to_id.get(candidate_rc)
                if neighbor_id is not None:
                    neighbors[marker_id].append(neighbor_id)
        return neighbors

    def _build_edges(self):
        edges = []
        for marker_id, neighbor_ids in self.neighbors.items():
            for neighbor_id in neighbor_ids:
                if marker_id < neighbor_id:
                    edges.append((marker_id, neighbor_id))
        return edges

    @staticmethod
    def _quat_to_rot(x, y, z, w):
        xx = x * x
        yy = y * y
        zz = z * z
        xy = x * y
        xz = x * z
        yz = y * z
        wx = w * x
        wy = w * y
        wz = w * z
        return np.asarray(
            [
                [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
                [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
                [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
            ],
            dtype=np.float64,
        )

    @staticmethod
    def _rot_to_quat(R):
        trace = float(R[0, 0] + R[1, 1] + R[2, 2])
        if trace > 0.0:
            s = np.sqrt(trace + 1.0) * 2.0
            qw = 0.25 * s
            qx = (R[2, 1] - R[1, 2]) / s
            qy = (R[0, 2] - R[2, 0]) / s
            qz = (R[1, 0] - R[0, 1]) / s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            qw = (R[2, 1] - R[1, 2]) / s
            qx = 0.25 * s
            qy = (R[0, 1] + R[1, 0]) / s
            qz = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
            qw = (R[0, 2] - R[2, 0]) / s
            qx = (R[0, 1] + R[1, 0]) / s
            qy = 0.25 * s
            qz = (R[1, 2] + R[2, 1]) / s
        else:
            s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
            qw = (R[1, 0] - R[0, 1]) / s
            qx = (R[0, 2] + R[2, 0]) / s
            qy = (R[1, 2] + R[2, 1]) / s
            qz = 0.25 * s
        return qx, qy, qz, qw

    @staticmethod
    def _normalize(vec, fallback=None):
        norm = np.linalg.norm(vec)
        if norm < 1e-8:
            return fallback
        return vec / norm

    def _pose_msg_to_point(self, pose_msg):
        position = pose_msg.pose.pose.position
        return np.asarray([position.x, position.y, position.z], dtype=np.float64)

    def _pose_msg_to_normal(self, pose_msg):
        normal = np.asarray([pose_msg.normal.x, pose_msg.normal.y, pose_msg.normal.z], dtype=np.float64)
        return self._normalize(normal, fallback=np.asarray([0.0, 0.0, 1.0], dtype=np.float64))

    def _compute_rigid_initialization(self, observed_points, observed_normals):
        all_points = {marker_id: self.id_to_center[marker_id].copy() for marker_id in self.layout_ids}
        if len(observed_points) >= 3:
            obs_ids = sorted(observed_points.keys())
            rest_pts = np.asarray([self.id_to_center[marker_id] for marker_id in obs_ids], dtype=np.float64)
            obs_pts = np.asarray([observed_points[marker_id] for marker_id in obs_ids], dtype=np.float64)
            rest_centroid = np.mean(rest_pts, axis=0)
            obs_centroid = np.mean(obs_pts, axis=0)
            rest_centered = rest_pts - rest_centroid
            obs_centered = obs_pts - obs_centroid
            H = np.matmul(rest_centered.T, obs_centered)
            try:
                U, _, Vt = np.linalg.svd(H, full_matrices=False)
                R = np.matmul(Vt.T, U.T)
            except np.linalg.LinAlgError:
                R = np.eye(3)
            if np.linalg.det(R) < 0.0:
                Vt[-1, :] *= -1.0
                R = np.matmul(Vt.T, U.T)
            t = obs_centroid - np.matmul(R, rest_centroid.reshape(3, 1)).reshape(3)
            for marker_id in self.layout_ids:
                all_points[marker_id] = np.matmul(R, self.id_to_center[marker_id].reshape(3, 1)).reshape(3) + t
            avg_normal = np.mean(np.asarray(list(observed_normals.values()), dtype=np.float64), axis=0)
            avg_normal = self._normalize(avg_normal, fallback=R[:, 2])
        elif observed_points:
            avg_point = np.mean(np.asarray(list(observed_points.values()), dtype=np.float64), axis=0)
            avg_rest = np.mean(np.asarray(list(self.id_to_center.values()), dtype=np.float64), axis=0)
            translation = avg_point - avg_rest
            for marker_id in self.layout_ids:
                all_points[marker_id] = self.id_to_center[marker_id] + translation
            avg_normal = np.mean(np.asarray(list(observed_normals.values()), dtype=np.float64), axis=0)
            avg_normal = self._normalize(avg_normal, fallback=np.asarray([0.0, 0.0, 1.0], dtype=np.float64))
        else:
            avg_normal = np.asarray([0.0, 0.0, 1.0], dtype=np.float64)
        return all_points, avg_normal

    def _solve_axis(self, axis_index, observed_points, measurement_weights, reference_points):
        num_nodes = len(self.layout_ids)
        rows = []
        rhs = []

        for marker_id, observed_point in observed_points.items():
            row = np.zeros(num_nodes, dtype=np.float64)
            row[self.id_to_index[marker_id]] = np.sqrt(measurement_weights[marker_id])
            rows.append(row)
            rhs.append(np.sqrt(measurement_weights[marker_id]) * observed_point[axis_index])

        for marker_id in self.layout_ids:
            row = np.zeros(num_nodes, dtype=np.float64)
            row[self.id_to_index[marker_id]] = np.sqrt(self.temporal_weight)
            rows.append(row)
            rhs.append(np.sqrt(self.temporal_weight) * reference_points[marker_id][axis_index])

        for marker_a, marker_b in self.edges:
            row = np.zeros(num_nodes, dtype=np.float64)
            row[self.id_to_index[marker_a]] = np.sqrt(self.edge_weight)
            row[self.id_to_index[marker_b]] = -np.sqrt(self.edge_weight)
            rows.append(row)
            target_delta = reference_points[marker_a][axis_index] - reference_points[marker_b][axis_index]
            rhs.append(np.sqrt(self.edge_weight) * target_delta)

        for marker_id in self.layout_ids:
            neighbor_ids = self.neighbors.get(marker_id, [])
            if not neighbor_ids:
                continue
            row = np.zeros(num_nodes, dtype=np.float64)
            row[self.id_to_index[marker_id]] = np.sqrt(self.smoothness_weight)
            coeff = -np.sqrt(self.smoothness_weight) / float(len(neighbor_ids))
            for neighbor_id in neighbor_ids:
                row[self.id_to_index[neighbor_id]] = coeff
            rows.append(row)
            neighbor_mean = np.mean(
                np.asarray([reference_points[neighbor_id] for neighbor_id in neighbor_ids], dtype=np.float64),
                axis=0,
            )
            laplacian_target = reference_points[marker_id][axis_index] - neighbor_mean[axis_index]
            rhs.append(np.sqrt(self.smoothness_weight) * laplacian_target)

        A = np.asarray(rows, dtype=np.float64)
        b = np.asarray(rhs, dtype=np.float64)
        solution, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
        return solution

    def _estimate_normals(self, estimated_points, observed_normals, reference_normal):
        normals = {}
        for marker_id in self.layout_ids:
            points = [estimated_points[marker_id]]
            for neighbor_id in self.neighbors.get(marker_id, []):
                points.append(estimated_points[neighbor_id])
            if len(points) >= 3:
                pts = np.asarray(points, dtype=np.float64)
                centered = pts - np.mean(pts, axis=0)
                try:
                    _, _, vh = np.linalg.svd(centered, full_matrices=False)
                    normal = self._normalize(vh[-1, :], fallback=reference_normal)
                except np.linalg.LinAlgError:
                    normal = reference_normal
            else:
                normal = reference_normal

            hint = observed_normals.get(marker_id, reference_normal)
            if np.dot(normal, hint) < 0.0:
                normal = -normal
            normals[marker_id] = normal
        return normals

    def _orientation_from_normal(self, normal, marker_id, estimated_points):
        normal = self._normalize(normal, fallback=np.asarray([0.0, 0.0, 1.0], dtype=np.float64))
        x_axis = None
        row, col = self.id_to_rowcol[marker_id]
        right_id = self.rowcol_to_id.get((row, col + 1))
        left_id = self.rowcol_to_id.get((row, col - 1))
        if right_id is not None:
            x_axis = estimated_points[right_id] - estimated_points[marker_id]
        elif left_id is not None:
            x_axis = estimated_points[marker_id] - estimated_points[left_id]
        if x_axis is None or np.linalg.norm(x_axis) < 1e-8:
            x_axis = np.asarray([1.0, 0.0, 0.0], dtype=np.float64)
        x_axis = x_axis - np.dot(x_axis, normal) * normal
        x_axis = self._normalize(x_axis, fallback=np.asarray([1.0, 0.0, 0.0], dtype=np.float64))
        y_axis = np.cross(normal, x_axis)
        y_axis = self._normalize(y_axis, fallback=np.asarray([0.0, 1.0, 0.0], dtype=np.float64))
        x_axis = np.cross(y_axis, normal)
        x_axis = self._normalize(x_axis, fallback=np.asarray([1.0, 0.0, 0.0], dtype=np.float64))
        return np.column_stack((x_axis, y_axis, normal))

    def _inflate_covariance(self, marker_id, observed_pose):
        if observed_pose is not None:
            return list(observed_pose.pose.covariance)

        covariance = np.asarray(
            self.prev_covariances.get(marker_id, np.zeros(36, dtype=np.float64)),
            dtype=np.float64,
        )
        covariance[0] = covariance[0] * self.prediction_variance_growth + self.missing_position_variance_step
        covariance[7] = covariance[7] * self.prediction_variance_growth + self.missing_position_variance_step
        covariance[14] = covariance[14] * self.prediction_variance_growth + self.missing_position_variance_step
        covariance[21] = covariance[21] * self.prediction_variance_growth + self.missing_orientation_variance_step
        covariance[28] = covariance[28] * self.prediction_variance_growth + self.missing_orientation_variance_step
        covariance[35] = covariance[35] * self.prediction_variance_growth + self.missing_orientation_variance_step
        return list(covariance)

    def _pose_cb(self, msg):
        if self.prev_frame_id is not None and msg.header.frame_id != self.prev_frame_id:
            self.prev_points = None
            self.prev_normals = None
            self.prev_covariances = {}
        self.prev_frame_id = msg.header.frame_id

        observed_pose_msgs = {int(pose.id): pose for pose in msg.poses}
        observed_points = {marker_id: self._pose_msg_to_point(pose) for marker_id, pose in observed_pose_msgs.items()}
        observed_normals = {marker_id: self._pose_msg_to_normal(pose) for marker_id, pose in observed_pose_msgs.items()}

        initial_points, initial_normal = self._compute_rigid_initialization(observed_points, observed_normals)
        if self.prev_points is None:
            reference_points = initial_points
            reference_normal = initial_normal
        else:
            reference_points = self.prev_points
            reference_normal = np.mean(np.asarray(list(self.prev_normals.values()), dtype=np.float64), axis=0)
            reference_normal = self._normalize(reference_normal, fallback=initial_normal)

        measurement_weights = {}
        for marker_id, pose_msg in observed_pose_msgs.items():
            covariance = np.asarray(pose_msg.pose.covariance, dtype=np.float64)
            pos_var = float(np.mean(covariance[[0, 7, 14]])) if covariance.size >= 15 else 1.0e-4
            pos_var = max(pos_var, 1.0e-6)
            measurement_weights[marker_id] = self.measurement_weight * max(0.1, pose_msg.detection_score) / pos_var

        estimated_points = {}
        for axis_index in range(3):
            axis_solution = self._solve_axis(axis_index, observed_points, measurement_weights, reference_points)
            for marker_id in self.layout_ids:
                if marker_id not in estimated_points:
                    estimated_points[marker_id] = np.zeros(3, dtype=np.float64)
                estimated_points[marker_id][axis_index] = axis_solution[self.id_to_index[marker_id]]

        estimated_normals = self._estimate_normals(estimated_points, observed_normals, reference_normal)

        output_msg = TagPoseArray()
        output_msg.header = msg.header
        output_msg.header.frame_id = msg.header.frame_id

        current_covariances = {}
        for marker_id in self.layout_ids:
            observed_pose = observed_pose_msgs.get(marker_id)
            covariance = self._inflate_covariance(marker_id, observed_pose)
            current_covariances[marker_id] = covariance

            R = self._orientation_from_normal(estimated_normals[marker_id], marker_id, estimated_points)
            qx, qy, qz, qw = self._rot_to_quat(R)

            pose_msg = TagPose()
            pose_msg.id = marker_id
            pose_msg.dictionary_name = observed_pose.dictionary_name if observed_pose is not None else self.layout_dict_name
            pose_msg.pose.pose.position.x = float(estimated_points[marker_id][0])
            pose_msg.pose.pose.position.y = float(estimated_points[marker_id][1])
            pose_msg.pose.pose.position.z = float(estimated_points[marker_id][2])
            pose_msg.pose.pose.orientation.x = qx
            pose_msg.pose.pose.orientation.y = qy
            pose_msg.pose.pose.orientation.z = qz
            pose_msg.pose.pose.orientation.w = qw
            pose_msg.pose.covariance = covariance
            pose_msg.normal.x = float(estimated_normals[marker_id][0])
            pose_msg.normal.y = float(estimated_normals[marker_id][1])
            pose_msg.normal.z = float(estimated_normals[marker_id][2])
            pose_msg.observed = observed_pose is not None
            if observed_pose is not None:
                pose_msg.tag_area_px = observed_pose.tag_area_px
                pose_msg.detection_score = observed_pose.detection_score
                pose_msg.reprojection_error_px = observed_pose.reprojection_error_px
                pose_msg.depth_fused = observed_pose.depth_fused
            else:
                pose_msg.tag_area_px = 0.0
                pose_msg.detection_score = 0.0
                pose_msg.reprojection_error_px = 0.0
                pose_msg.depth_fused = False
            output_msg.poses.append(pose_msg)

        self.prev_points = {marker_id: estimated_points[marker_id].copy() for marker_id in self.layout_ids}
        self.prev_normals = {marker_id: estimated_normals[marker_id].copy() for marker_id in self.layout_ids}
        self.prev_covariances = current_covariances
        self.pose_pub.publish(output_msg)


if __name__ == "__main__":
    try:
        PaperSurfaceEstimator()
        rospy.spin()
    except Exception as exc:
        rospy.logfatal("paper_surface_estimator failed: %s", str(exc))
