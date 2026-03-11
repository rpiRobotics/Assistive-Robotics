#!/usr/bin/env python3

from __future__ import print_function

import json
import os
import threading

import cv2
import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from assistive_msgs.msg import TagDepthObservation
from assistive_msgs.msg import TagDepthObservationArray
from assistive_msgs.msg import TagDetectionArray
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2


class PaperDepthProcessor(object):
    def __init__(self):
        rospy.init_node("paper_depth_processor", anonymous=False)

        self.layout_json_path = os.path.expanduser(rospy.get_param("~layout_json_path", ""))
        if not self.layout_json_path:
            rospy.logfatal("~layout_json_path must be set.")
            raise RuntimeError("Missing layout_json_path")

        self.detection_topic_name = rospy.get_param("~detection_topic_name", "paper_tag_detections")
        self.camera_info_topic_name = rospy.get_param("~camera_info_topic_name", "/nuc/rgb/camera_info")
        self.depth_image_topic_name = rospy.get_param("~depth_image_topic_name", "/nuc/depth_to_rgb/image_raw")
        self.depth_observation_topic_name = rospy.get_param(
            "~depth_observation_topic_name", "paper_tag_depth_observations"
        )
        self.mask_topic_name = rospy.get_param("~mask_topic_name", "paper_sheet_mask")
        self.point_cloud_topic_name = rospy.get_param("~point_cloud_topic_name", "paper_sheet_points")
        self.camera_frame_id = rospy.get_param("~camera_frame_id", "")

        self.depth_unit_scale = float(rospy.get_param("~depth_unit_scale", 0.001))
        self.depth_max_age_sec = float(rospy.get_param("~depth_max_age_sec", 0.15))
        self.depth_patch_radius_px = int(rospy.get_param("~depth_patch_radius_px", 2))
        self.plane_fit_stride_px = max(1, int(rospy.get_param("~plane_fit_stride_px", 2)))
        self.plane_fit_min_points = max(3, int(rospy.get_param("~plane_fit_min_points", 20)))
        self.sheet_depth_margin_m = float(rospy.get_param("~sheet_depth_margin_m", 0.04))
        self.point_cloud_stride_px = max(1, int(rospy.get_param("~point_cloud_stride_px", 4)))
        self.min_support_ratio = float(rospy.get_param("~min_support_ratio", 0.15))
        self.mask_morph_kernel_px = max(1, int(rospy.get_param("~mask_morph_kernel_px", 5)))

        self._load_layout(self.layout_json_path)

        self.bridge = CvBridge()
        self.data_lock = threading.Lock()
        self.depth_lock = threading.Lock()
        self.K = None
        self.camera_info_ready = False
        self.latest_depth = None
        self.latest_depth_stamp = None

        self.depth_observation_pub = rospy.Publisher(
            self.depth_observation_topic_name, TagDepthObservationArray, queue_size=1
        )
        self.mask_pub = rospy.Publisher(self.mask_topic_name, Image, queue_size=1)
        self.point_cloud_pub = rospy.Publisher(self.point_cloud_topic_name, PointCloud2, queue_size=1)

        self.camera_info_sub = rospy.Subscriber(
            self.camera_info_topic_name, CameraInfo, self._camera_info_cb, queue_size=1
        )
        self.depth_sub = rospy.Subscriber(self.depth_image_topic_name, Image, self._depth_cb, queue_size=1)
        self.detection_sub = rospy.Subscriber(
            self.detection_topic_name, TagDetectionArray, self._detection_cb, queue_size=1
        )

        rospy.loginfo("paper_depth_processor initialized.")
        rospy.loginfo("Publishing depth observations on: %s", self.depth_observation_topic_name)

    def _load_layout(self, path):
        if not os.path.exists(path):
            raise IOError("Layout json not found: {}".format(path))
        with open(path, "r") as f:
            layout_data = json.load(f)
        if not layout_data.get("markers"):
            raise ValueError("Layout has no markers")

    def _camera_info_cb(self, msg):
        with self.data_lock:
            self.K = np.asarray(msg.K, dtype=np.float64).reshape(3, 3)
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

    def _depth_value_to_m(self, val):
        if np.isnan(val) or np.isinf(val):
            return None
        if val <= 0:
            return None
        return float(val) * self.depth_unit_scale

    @staticmethod
    def _pixel_to_cam(K, u, v, z_m):
        fx = K[0, 0]
        fy = K[1, 1]
        cx = K[0, 2]
        cy = K[1, 2]
        x = (u - cx) * z_m / fx
        y = (v - cy) * z_m / fy
        return np.asarray([x, y, z_m], dtype=np.float64)

    @staticmethod
    def _normalize(vec, fallback=None):
        norm = np.linalg.norm(vec)
        if norm < 1e-8:
            return fallback
        return vec / norm

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

        values_m = []
        for val in patch.reshape(-1):
            depth_m = self._depth_value_to_m(float(val))
            if depth_m is not None:
                values_m.append(depth_m)
        if not values_m:
            return None
        return float(np.median(np.asarray(values_m, dtype=np.float64)))

    def _fit_depth_plane(self, depth_img, corners_px, K):
        h, w = depth_img.shape[:2]
        xs = corners_px[:, 0]
        ys = corners_px[:, 1]
        u0 = max(0, int(np.floor(np.min(xs))))
        u1 = min(w - 1, int(np.ceil(np.max(xs))))
        v0 = max(0, int(np.floor(np.min(ys))))
        v1 = min(h - 1, int(np.ceil(np.max(ys))))
        if u1 <= u0 or v1 <= v0:
            return None

        polygon = corners_px.astype(np.float32)
        points = []
        attempted_count = 0
        for v in range(v0, v1 + 1, self.plane_fit_stride_px):
            for u in range(u0, u1 + 1, self.plane_fit_stride_px):
                if cv2.pointPolygonTest(polygon, (float(u), float(v)), False) < 0:
                    continue
                attempted_count += 1
                depth_m = self._depth_value_to_m(float(depth_img[v, u]))
                if depth_m is None:
                    continue
                points.append(self._pixel_to_cam(K, u, v, depth_m))

        valid_ratio = float(len(points)) / float(max(1, attempted_count))
        if len(points) < self.plane_fit_min_points or valid_ratio < self.min_support_ratio:
            return {
                "center_cam": None,
                "normal_cam": None,
                "mean_depth_m": 0.0,
                "valid_point_ratio": valid_ratio,
                "support_point_count": len(points),
                "plane_fitted": False,
            }

        pts = np.asarray(points, dtype=np.float64)
        centroid = np.mean(pts, axis=0)
        centered = pts - centroid
        try:
            _, _, vh = np.linalg.svd(centered, full_matrices=False)
        except np.linalg.LinAlgError:
            return {
                "center_cam": None,
                "normal_cam": None,
                "mean_depth_m": float(np.mean(pts[:, 2])),
                "valid_point_ratio": valid_ratio,
                "support_point_count": len(points),
                "plane_fitted": False,
            }

        normal = self._normalize(vh[-1, :], fallback=np.asarray([0.0, 0.0, -1.0], dtype=np.float64))
        if np.dot(normal, centroid) > 0.0:
            normal = -normal
        return {
            "center_cam": centroid,
            "normal_cam": normal,
            "mean_depth_m": float(np.mean(pts[:, 2])),
            "valid_point_ratio": valid_ratio,
            "support_point_count": len(points),
            "plane_fitted": True,
        }

    def _build_sheet_mask(self, depth_img, detections):
        h, w = depth_img.shape[:2]
        if not detections:
            return np.zeros((h, w), dtype=np.uint8), []

        hull_points = []
        tag_depths = []
        for detection in detections:
            corners_px = np.asarray([[pt.x, pt.y] for pt in detection.corners_px], dtype=np.float32)
            if corners_px.shape != (4, 2):
                continue
            hull_points.append(corners_px)
            center_px = np.mean(corners_px, axis=0)
            center_depth_m = self._sample_depth_m(depth_img, center_px[0], center_px[1])
            if center_depth_m is not None:
                tag_depths.append(center_depth_m)

        if not hull_points:
            return np.zeros((h, w), dtype=np.uint8), []

        hull = cv2.convexHull(np.vstack(hull_points))
        base_mask = np.zeros((h, w), dtype=np.uint8)
        cv2.fillConvexPoly(base_mask, hull.astype(np.int32), 255)

        median_depth_m = float(np.median(np.asarray(tag_depths, dtype=np.float64))) if tag_depths else None
        valid_mask = np.zeros((h, w), dtype=np.uint8)
        yy, xx = np.nonzero(base_mask)
        if median_depth_m is not None:
            for v, u in zip(yy.tolist(), xx.tolist()):
                depth_m = self._depth_value_to_m(float(depth_img[v, u]))
                if depth_m is None:
                    continue
                if abs(depth_m - median_depth_m) <= self.sheet_depth_margin_m:
                    valid_mask[v, u] = 255

        if np.count_nonzero(valid_mask) == 0:
            valid_mask = base_mask

        kernel = np.ones((self.mask_morph_kernel_px, self.mask_morph_kernel_px), dtype=np.uint8)
        valid_mask = cv2.morphologyEx(valid_mask, cv2.MORPH_CLOSE, kernel)
        valid_mask = cv2.morphologyEx(valid_mask, cv2.MORPH_OPEN, kernel)
        return valid_mask, tag_depths

    def _mask_to_point_cloud(self, mask, depth_img, K):
        points = []
        h, w = mask.shape[:2]
        for v in range(0, h, self.point_cloud_stride_px):
            for u in range(0, w, self.point_cloud_stride_px):
                if mask[v, u] == 0:
                    continue
                depth_m = self._depth_value_to_m(float(depth_img[v, u]))
                if depth_m is None:
                    continue
                point = self._pixel_to_cam(K, u, v, depth_m)
                points.append([float(point[0]), float(point[1]), float(point[2])])
        return points

    @staticmethod
    def _to_point(vec):
        msg = Point()
        msg.x = float(vec[0])
        msg.y = float(vec[1])
        msg.z = float(vec[2])
        return msg

    @staticmethod
    def _to_vector3(vec):
        msg = Vector3()
        msg.x = float(vec[0])
        msg.y = float(vec[1])
        msg.z = float(vec[2])
        return msg

    def _detection_cb(self, msg):
        with self.data_lock:
            if not self.camera_info_ready:
                return
            K = self.K.copy()
            camera_frame_id = self.camera_frame_id

        with self.depth_lock:
            if self.latest_depth is None or self.latest_depth_stamp is None:
                return
            depth_img = self.latest_depth.copy()
            depth_stamp = self.latest_depth_stamp

        stamp = msg.header.stamp if msg.header.stamp != rospy.Time(0) else rospy.Time.now()
        age = abs((stamp - depth_stamp).to_sec())
        if age > self.depth_max_age_sec:
            rospy.logwarn_throttle(2.0, "Skipping depth processing, depth age %.3fs exceeds limit.", age)
            return

        header = msg.header
        if camera_frame_id:
            header.frame_id = camera_frame_id

        observations_msg = TagDepthObservationArray()
        observations_msg.header = header

        valid_detections = []
        for detection in msg.detections:
            if len(detection.corners_px) != 4:
                continue
            valid_detections.append(detection)
            corners_px = np.asarray([[pt.x, pt.y] for pt in detection.corners_px], dtype=np.float64)
            plane_result = self._fit_depth_plane(depth_img, corners_px, K)
            center_cam = plane_result["center_cam"]
            normal_cam = plane_result["normal_cam"]

            if center_cam is None:
                center_px = np.mean(corners_px, axis=0)
                center_depth_m = self._sample_depth_m(depth_img, center_px[0], center_px[1])
                if center_depth_m is not None:
                    center_cam = self._pixel_to_cam(K, center_px[0], center_px[1], center_depth_m)
                    plane_result["mean_depth_m"] = center_depth_m
            if center_cam is None:
                continue

            if normal_cam is None:
                normal_cam = np.asarray([0.0, 0.0, -1.0], dtype=np.float64)

            observation = TagDepthObservation()
            observation.id = int(detection.id)
            observation.center_cam = self._to_point(center_cam)
            observation.normal_cam = self._to_vector3(normal_cam)
            observation.mean_depth_m = float(plane_result["mean_depth_m"])
            observation.valid_point_ratio = float(plane_result["valid_point_ratio"])
            observation.support_point_count = int(plane_result["support_point_count"])
            observation.plane_fitted = bool(plane_result["plane_fitted"])
            observations_msg.observations.append(observation)

        observations_msg.observations.sort(key=lambda obs: obs.id)
        self.depth_observation_pub.publish(observations_msg)

        mask, _ = self._build_sheet_mask(depth_img, valid_detections)
        try:
            mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding="mono8")
            mask_msg.header = header
            self.mask_pub.publish(mask_msg)
        except CvBridgeError:
            rospy.logwarn_throttle(5.0, "Failed to publish paper sheet mask.")

        cloud_points = self._mask_to_point_cloud(mask, depth_img, K)
        cloud_msg = pc2.create_cloud_xyz32(header, cloud_points)
        self.point_cloud_pub.publish(cloud_msg)


if __name__ == "__main__":
    try:
        PaperDepthProcessor()
        rospy.spin()
    except Exception as exc:
        rospy.logfatal("paper_depth_processor failed: %s", str(exc))
