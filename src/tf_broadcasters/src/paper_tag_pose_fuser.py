#!/usr/bin/env python3

from __future__ import print_function

import json
import os
import threading

import cv2
import numpy as np
import rospy
import tf2_ros
from assistive_msgs.msg import TagDepthObservationArray
from assistive_msgs.msg import TagDetectionArray
from assistive_msgs.msg import TagPose
from assistive_msgs.msg import TagPoseArray
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image


class PaperTagPoseFuser(object):
    def __init__(self):
        rospy.init_node("paper_tag_pose_fuser", anonymous=False)

        self.layout_json_path = os.path.expanduser(rospy.get_param("~layout_json_path", ""))
        if not self.layout_json_path:
            rospy.logfatal("~layout_json_path must be set.")
            raise RuntimeError("Missing layout_json_path")

        self.detection_topic_name = rospy.get_param("~detection_topic_name", "paper_tag_detections")
        self.depth_observation_topic_name = rospy.get_param(
            "~depth_observation_topic_name", "paper_tag_depth_observations"
        )
        self.pose_topic_name = rospy.get_param("~pose_topic_name", "paper_tag_poses")
        self.camera_info_topic_name = rospy.get_param("~camera_info_topic_name", "/nuc/rgb/camera_info")
        self.camera_frame_id = rospy.get_param("~camera_frame_id", "")
        self.target_frame_id = rospy.get_param("~target_frame_id", "")
        self.use_depth_observations = rospy.get_param("~use_depth_observations", True)
        self.depth_observation_max_age_sec = float(rospy.get_param("~depth_observation_max_age_sec", 0.15))
        self.depth_image_topic_name = rospy.get_param("~depth_image_topic_name", "")
        self.use_depth_refinement = rospy.get_param("~use_depth_refinement", True)
        self.depth_unit_scale = float(rospy.get_param("~depth_unit_scale", 0.001))
        self.depth_max_age_sec = float(rospy.get_param("~depth_max_age_sec", 0.15))
        self.depth_patch_radius_px = int(rospy.get_param("~depth_patch_radius_px", 2))
        self.plane_fit_stride_px = max(1, int(rospy.get_param("~plane_fit_stride_px", 2)))
        self.plane_fit_min_points = max(3, int(rospy.get_param("~plane_fit_min_points", 20)))
        self.tf_lookup_timeout_sec = float(rospy.get_param("~tf_lookup_timeout_sec", 0.05))
        self.pnp_reprojection_error = float(rospy.get_param("~pnp_reprojection_error", 3.0))

        self.base_position_variance = float(rospy.get_param("~base_position_variance", 1.0e-5))
        self.base_orientation_variance = float(rospy.get_param("~base_orientation_variance", 2.5e-3))
        self.range_position_variance_scale = float(rospy.get_param("~range_position_variance_scale", 0.03))
        self.angle_position_variance_scale = float(rospy.get_param("~angle_position_variance_scale", 0.08))
        self.range_orientation_variance_scale = float(rospy.get_param("~range_orientation_variance_scale", 0.05))
        self.angle_orientation_variance_scale = float(rospy.get_param("~angle_orientation_variance_scale", 0.2))
        self.depth_fused_variance_scale = float(rospy.get_param("~depth_fused_variance_scale", 0.6))

        self.layout_data = self._load_layout(self.layout_json_path)
        self.default_marker_size_m = float(self.layout_data.get("grid", {}).get("marker_mm", 60.0)) / 1000.0
        self.id_to_size_m = self._read_marker_sizes(self.layout_data)
        self.obj_corners_cache = {}

        self.pnp_square_flag = cv2.SOLVEPNP_ITERATIVE
        if hasattr(cv2, "SOLVEPNP_IPPE_SQUARE"):
            self.pnp_square_flag = cv2.SOLVEPNP_IPPE_SQUARE

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.data_lock = threading.Lock()
        self.depth_lock = threading.Lock()
        self.K = None
        self.D = None
        self.camera_info_ready = False
        self.latest_depth = None
        self.latest_depth_stamp = None
        self.latest_depth_observations = None
        self.latest_depth_observations_stamp = None

        self.pose_pub = rospy.Publisher(self.pose_topic_name, TagPoseArray, queue_size=1)
        self.camera_info_sub = rospy.Subscriber(
            self.camera_info_topic_name, CameraInfo, self._camera_info_cb, queue_size=1
        )
        self.detection_sub = rospy.Subscriber(
            self.detection_topic_name, TagDetectionArray, self._detection_cb, queue_size=1
        )
        self.depth_observation_sub = None
        if self.depth_observation_topic_name:
            self.depth_observation_sub = rospy.Subscriber(
                self.depth_observation_topic_name,
                TagDepthObservationArray,
                self._depth_observation_cb,
                queue_size=1,
            )
        self.depth_sub = None
        if self.depth_image_topic_name:
            self.depth_sub = rospy.Subscriber(self.depth_image_topic_name, Image, self._depth_cb, queue_size=1)

        rospy.loginfo("paper_tag_pose_fuser initialized.")
        rospy.loginfo("Publishing per-tag poses on: %s", self.pose_topic_name)

    def _load_layout(self, path):
        if not os.path.exists(path):
            raise IOError("Layout json not found: {}".format(path))
        with open(path, "r") as f:
            return json.load(f)

    def _read_marker_sizes(self, layout_data):
        sizes = {}
        for marker in layout_data.get("markers", []):
            marker_id = int(marker["id"])
            size_m = float(marker.get("marker_length_mm", self.default_marker_size_m * 1000.0)) / 1000.0
            sizes[marker_id] = size_m
        if not sizes:
            raise ValueError("Layout has no markers")
        return sizes

    def _camera_info_cb(self, msg):
        with self.data_lock:
            self.K = np.asarray(msg.K, dtype=np.float64).reshape(3, 3)
            self.D = np.asarray(msg.D, dtype=np.float64)
            self.camera_info_ready = True
            if not self.camera_frame_id:
                self.camera_frame_id = msg.header.frame_id

    def _depth_cb(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except CvBridgeError:
            return
        with self.depth_lock:
            self.latest_depth = depth
            self.latest_depth_stamp = msg.header.stamp

    def _depth_observation_cb(self, msg):
        observations = {}
        for observation in msg.observations:
            observations[int(observation.id)] = observation
        with self.depth_lock:
            self.latest_depth_observations = observations
            self.latest_depth_observations_stamp = msg.header.stamp

    def _marker_object_corners(self, marker_size_m):
        key = round(float(marker_size_m), 6)
        if key in self.obj_corners_cache:
            return self.obj_corners_cache[key]

        half = key * 0.5
        obj = np.asarray(
            [
                [-half, half, 0.0],
                [half, half, 0.0],
                [half, -half, 0.0],
                [-half, -half, 0.0],
            ],
            dtype=np.float64,
        )
        self.obj_corners_cache[key] = obj
        return obj

    def _estimate_marker_pose(self, marker_corners_img, marker_size_m, K, D):
        obj = self._marker_object_corners(marker_size_m)
        success, rvec, tvec = cv2.solvePnP(
            obj,
            marker_corners_img,
            K,
            D,
            flags=self.pnp_square_flag,
        )
        if not success and self.pnp_square_flag != cv2.SOLVEPNP_ITERATIVE:
            success, rvec, tvec = cv2.solvePnP(
                obj,
                marker_corners_img,
                K,
                D,
                flags=cv2.SOLVEPNP_ITERATIVE,
            )
        if not success:
            return None, None, None, None

        projected, _ = cv2.projectPoints(obj, rvec, tvec, K, D)
        reproj_error = np.mean(np.linalg.norm(projected.reshape(-1, 2) - marker_corners_img, axis=1))
        if reproj_error > self.pnp_reprojection_error:
            return None, None, None, None

        center_cam = np.asarray(tvec, dtype=np.float64).reshape(3)
        return center_cam, rvec, tvec, float(reproj_error)

    @staticmethod
    def _normalize(vec, fallback=None):
        norm = np.linalg.norm(vec)
        if norm < 1e-8:
            return fallback
        return vec / norm

    def _depth_value_to_m(self, val):
        if np.isnan(val) or np.isinf(val):
            return None
        if val <= 0:
            return None
        return float(val) * self.depth_unit_scale

    def _sample_depth_m(self, depth_img, u, v):
        h, w = depth_img.shape[:2]
        uu = int(round(u))
        vv = int(round(v))
        if uu < 0 or uu >= w or vv < 0 or vv >= h:
            return None

        r = max(0, self.depth_patch_radius_px)
        u0 = max(0, uu - r)
        u1 = min(w, uu + r + 1)
        v0 = max(0, vv - r)
        v1 = min(h, vv + r + 1)
        patch = depth_img[v0:v1, u0:u1]
        if patch.size == 0:
            return None

        depth_vals = []
        for val in patch.reshape(-1):
            depth_m = self._depth_value_to_m(float(val))
            if depth_m is not None:
                depth_vals.append(depth_m)
        if not depth_vals:
            return None
        return float(np.median(np.asarray(depth_vals, dtype=np.float64)))

    @staticmethod
    def _pixel_to_cam(K, u, v, z_m):
        fx = K[0, 0]
        fy = K[1, 1]
        cx = K[0, 2]
        cy = K[1, 2]
        x = (u - cx) * z_m / fx
        y = (v - cy) * z_m / fy
        return np.asarray([x, y, z_m], dtype=np.float64)

    def _fit_depth_plane(self, depth_img, corners_px, K):
        h, w = depth_img.shape[:2]
        xs = corners_px[:, 0]
        ys = corners_px[:, 1]
        u0 = max(0, int(np.floor(np.min(xs))))
        u1 = min(w - 1, int(np.ceil(np.max(xs))))
        v0 = max(0, int(np.floor(np.min(ys))))
        v1 = min(h - 1, int(np.ceil(np.max(ys))))
        if u1 <= u0 or v1 <= v0:
            return None, None

        points = []
        polygon = corners_px.astype(np.float32)
        for v in range(v0, v1 + 1, self.plane_fit_stride_px):
            for u in range(u0, u1 + 1, self.plane_fit_stride_px):
                if cv2.pointPolygonTest(polygon, (float(u), float(v)), False) < 0:
                    continue
                depth_m = self._depth_value_to_m(float(depth_img[v, u]))
                if depth_m is None:
                    continue
                points.append(self._pixel_to_cam(K, u, v, depth_m))

        if len(points) < self.plane_fit_min_points:
            return None, None

        pts = np.asarray(points, dtype=np.float64)
        centroid = np.mean(pts, axis=0)
        centered = pts - centroid
        try:
            _, _, vh = np.linalg.svd(centered, full_matrices=False)
        except np.linalg.LinAlgError:
            return None, None
        normal = self._normalize(vh[-1, :])
        if normal is None:
            return None, None

        # Orient the normal to point toward the camera.
        if np.dot(normal, centroid) > 0.0:
            normal = -normal
        return centroid, normal

    def _camera_to_target_transform(self, camera_frame_id, target_frame_id, stamp):
        if target_frame_id == camera_frame_id:
            return np.eye(3), np.zeros((3, 1))
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                target_frame_id,
                camera_frame_id,
                stamp,
                rospy.Duration(self.tf_lookup_timeout_sec),
            )
        except Exception as exc:
            rospy.logwarn_throttle(
                2.0,
                "TF lookup failed: %s -> %s (%s)",
                target_frame_id,
                camera_frame_id,
                str(exc),
            )
            return None, None

        q = tf_msg.transform.rotation
        t = tf_msg.transform.translation
        R = self._quat_to_rot(q.x, q.y, q.z, q.w)
        T = np.asarray([[t.x], [t.y], [t.z]], dtype=np.float64)
        return R, T

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

    def _rotation_from_normal_and_hint(self, normal, x_hint, y_hint):
        z_axis = self._normalize(normal)
        if z_axis is None:
            return None

        x_axis = x_hint - np.dot(x_hint, z_axis) * z_axis
        x_axis = self._normalize(x_axis)
        if x_axis is None:
            x_axis = np.cross(y_hint, z_axis)
            x_axis = self._normalize(x_axis)
        if x_axis is None:
            return None

        y_axis = np.cross(z_axis, x_axis)
        y_axis = self._normalize(y_axis)
        if y_axis is None:
            return None

        x_axis = np.cross(y_axis, z_axis)
        x_axis = self._normalize(x_axis)
        if x_axis is None:
            return None
        return np.column_stack((x_axis, y_axis, z_axis))

    def _compute_covariance(self, center_cam, normal_cam, depth_fused):
        range_m = float(np.linalg.norm(center_cam))
        view_alignment = 1.0
        if range_m > 1e-8:
            ray_dir = center_cam / range_m
            view_alignment = abs(float(np.dot(normal_cam, -ray_dir)))
        angle_penalty = max(0.0, 1.0 - view_alignment)

        pos_var = self.base_position_variance * (
            1.0
            + self.range_position_variance_scale * range_m * range_m
            + self.angle_position_variance_scale * angle_penalty * angle_penalty
        )
        rot_var = self.base_orientation_variance * (
            1.0
            + self.range_orientation_variance_scale * range_m * range_m
            + self.angle_orientation_variance_scale * angle_penalty * angle_penalty
        )
        if depth_fused:
            pos_var *= self.depth_fused_variance_scale
            rot_var *= self.depth_fused_variance_scale

        covariance = np.zeros(36, dtype=np.float64)
        covariance[0] = pos_var
        covariance[7] = pos_var
        covariance[14] = pos_var
        covariance[21] = rot_var
        covariance[28] = rot_var
        covariance[35] = rot_var
        return list(covariance)

    def _detection_cb(self, msg):
        with self.data_lock:
            if not self.camera_info_ready:
                return
            K = self.K.copy()
            D = self.D.copy()
            camera_frame_id = self.camera_frame_id

        if not camera_frame_id:
            rospy.logwarn_throttle(5.0, "Camera frame is empty; waiting for CameraInfo.")
            return

        stamp = msg.header.stamp if msg.header.stamp != rospy.Time(0) else rospy.Time.now()
        target_frame = self.target_frame_id if self.target_frame_id else camera_frame_id
        R_tc, t_tc = self._camera_to_target_transform(camera_frame_id, target_frame, stamp)
        if R_tc is None:
            return

        depth_img = None
        depth_stamp = None
        depth_observations = None
        depth_observations_stamp = None
        if self.use_depth_refinement:
            with self.depth_lock:
                depth_observations = self.latest_depth_observations
                depth_observations_stamp = self.latest_depth_observations_stamp
                if self.latest_depth is not None:
                    depth_img = self.latest_depth.copy()
                    depth_stamp = self.latest_depth_stamp

        pose_array = TagPoseArray()
        pose_array.header = msg.header
        pose_array.header.frame_id = target_frame

        for detection in msg.detections:
            marker_id = int(detection.id)
            marker_size_m = self.id_to_size_m.get(marker_id, self.default_marker_size_m)
            if len(detection.corners_px) != 4:
                continue

            corners_px = np.asarray([[pt.x, pt.y] for pt in detection.corners_px], dtype=np.float64)
            center_cam, rvec, _, reproj_error = self._estimate_marker_pose(corners_px, marker_size_m, K, D)
            if center_cam is None:
                continue

            R_cm, _ = cv2.Rodrigues(rvec)
            normal_cam = self._normalize(R_cm[:, 2], fallback=np.asarray([0.0, 0.0, -1.0], dtype=np.float64))
            refined_center_cam = center_cam.copy()
            refined_normal_cam = normal_cam.copy()
            refined_rotation_cam = R_cm.copy()
            depth_fused = False

            if (
                self.use_depth_refinement
                and self.use_depth_observations
                and depth_observations is not None
                and depth_observations_stamp is not None
            ):
                obs_age = abs((stamp - depth_observations_stamp).to_sec())
                depth_observation = depth_observations.get(marker_id)
                if depth_observation is not None and obs_age <= self.depth_observation_max_age_sec:
                    refined_center_cam = np.asarray(
                        [
                            depth_observation.center_cam.x,
                            depth_observation.center_cam.y,
                            depth_observation.center_cam.z,
                        ],
                        dtype=np.float64,
                    )
                    refined_normal_cam = self._normalize(
                        np.asarray(
                            [
                                depth_observation.normal_cam.x,
                                depth_observation.normal_cam.y,
                                depth_observation.normal_cam.z,
                            ],
                            dtype=np.float64,
                        ),
                        fallback=refined_normal_cam,
                    )
                    refined_rotation_cam = self._rotation_from_normal_and_hint(
                        refined_normal_cam,
                        R_cm[:, 0],
                        R_cm[:, 1],
                    )
                    if refined_rotation_cam is None:
                        refined_rotation_cam = R_cm.copy()
                    depth_fused = True

            if (
                self.use_depth_refinement
                and not depth_fused
                and depth_img is not None
                and depth_stamp is not None
            ):
                age = abs((stamp - depth_stamp).to_sec())
                if age <= self.depth_max_age_sec:
                    plane_center_cam, plane_normal_cam = self._fit_depth_plane(depth_img, corners_px, K)
                    if plane_center_cam is not None and plane_normal_cam is not None:
                        if np.dot(plane_normal_cam, refined_normal_cam) < 0.0:
                            plane_normal_cam = -plane_normal_cam
                        refined_center_cam = plane_center_cam
                        refined_normal_cam = plane_normal_cam
                        refined_rotation_cam = self._rotation_from_normal_and_hint(
                            refined_normal_cam,
                            R_cm[:, 0],
                            R_cm[:, 1],
                        )
                        if refined_rotation_cam is None:
                            refined_rotation_cam = R_cm.copy()
                        depth_fused = True
                    else:
                        center_px = np.mean(corners_px, axis=0)
                        center_depth_m = self._sample_depth_m(depth_img, center_px[0], center_px[1])
                        if center_depth_m is not None:
                            refined_center_cam = self._pixel_to_cam(K, center_px[0], center_px[1], center_depth_m)
                            depth_fused = True

            center_target = (np.matmul(R_tc, refined_center_cam.reshape(3, 1)) + t_tc).reshape(3)
            normal_target = np.matmul(R_tc, refined_normal_cam.reshape(3, 1)).reshape(3)
            normal_target = self._normalize(normal_target, fallback=np.asarray([0.0, 0.0, 1.0], dtype=np.float64))
            R_tm = np.matmul(R_tc, refined_rotation_cam)
            qx, qy, qz, qw = self._rot_to_quat(R_tm)

            pose_msg = TagPose()
            pose_msg.id = marker_id
            pose_msg.dictionary_name = detection.dictionary_name
            pose_msg.pose.pose.position.x = float(center_target[0])
            pose_msg.pose.pose.position.y = float(center_target[1])
            pose_msg.pose.pose.position.z = float(center_target[2])
            pose_msg.pose.pose.orientation.x = qx
            pose_msg.pose.pose.orientation.y = qy
            pose_msg.pose.pose.orientation.z = qz
            pose_msg.pose.pose.orientation.w = qw
            pose_msg.pose.covariance = self._compute_covariance(refined_center_cam, refined_normal_cam, depth_fused)
            pose_msg.normal.x = float(normal_target[0])
            pose_msg.normal.y = float(normal_target[1])
            pose_msg.normal.z = float(normal_target[2])
            pose_msg.observed = True
            pose_msg.tag_area_px = detection.tag_area_px
            pose_msg.detection_score = detection.detection_score
            pose_msg.reprojection_error_px = reproj_error
            pose_msg.depth_fused = depth_fused
            pose_array.poses.append(pose_msg)

        pose_array.poses.sort(key=lambda pose: pose.id)
        self.pose_pub.publish(pose_array)


if __name__ == "__main__":
    try:
        PaperTagPoseFuser()
        rospy.spin()
    except Exception as exc:
        rospy.logfatal("paper_tag_pose_fuser failed: %s", str(exc))
