#!/usr/bin/env python3

from __future__ import print_function

import json
import os

import cv2
import numpy as np
import rospy
from assistive_msgs.msg import TagDetection
from assistive_msgs.msg import TagDetectionArray
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point32
from sensor_msgs.msg import Image


ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
}


class PaperTagDetector(object):
    def __init__(self):
        rospy.init_node("paper_tag_detector", anonymous=False)

        self.layout_json_path = os.path.expanduser(rospy.get_param("~layout_json_path", ""))
        if not self.layout_json_path:
            rospy.logfatal("~layout_json_path must be set.")
            raise RuntimeError("Missing layout_json_path")

        self.image_topic_name = rospy.get_param("~image_topic_name", "/nuc/rgb/image_raw")
        self.detection_topic_name = rospy.get_param("~detection_topic_name", "paper_tag_detections")
        self.publish_debug_image = rospy.get_param("~publish_debug_image", True)
        self.debug_image_topic_name = rospy.get_param("~debug_image_topic_name", "paper_tag_debug_image")
        self.min_tag_area_px = float(rospy.get_param("~min_tag_area_px", 0.0))
        self.quality_half_area_px = float(rospy.get_param("~quality_half_area_px", 2500.0))

        layout_data = self._load_layout(self.layout_json_path)
        self.layout_marker_ids = self._read_layout_marker_ids(layout_data)
        self.aruco_dict_name = rospy.get_param(
            "~aruco_dict_name", layout_data.get("grid", {}).get("dict", "DICT_4X4_50")
        )
        self.aruco_dict = self._make_aruco_dict(self.aruco_dict_name)
        self.aruco_params = self._make_aruco_params()
        self.aruco_detector = None
        if hasattr(cv2.aruco, "ArucoDetector"):
            self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.bridge = CvBridge()
        self.detection_pub = rospy.Publisher(self.detection_topic_name, TagDetectionArray, queue_size=1)
        self.debug_pub = None
        if self.publish_debug_image:
            self.debug_pub = rospy.Publisher(self.debug_image_topic_name, Image, queue_size=1)

        self.image_sub = rospy.Subscriber(self.image_topic_name, Image, self._image_cb, queue_size=1)

        rospy.loginfo("paper_tag_detector initialized.")
        rospy.loginfo("Layout markers loaded: %d", len(self.layout_marker_ids))
        rospy.loginfo("Publishing detections on: %s", self.detection_topic_name)

    def _load_layout(self, path):
        if not os.path.exists(path):
            raise IOError("Layout json not found: {}".format(path))
        with open(path, "r") as f:
            return json.load(f)

    @staticmethod
    def _read_layout_marker_ids(layout_data):
        markers = layout_data.get("markers", [])
        if not markers:
            raise ValueError("Layout has no markers")
        return set(int(marker["id"]) for marker in markers)

    @staticmethod
    def _polygon_area_px(points):
        return abs(float(cv2.contourArea(points.astype(np.float32))))

    def _compute_detection_score(self, area_px):
        if area_px <= 0.0:
            return 0.0
        return float(area_px / (area_px + self.quality_half_area_px))

    def _make_aruco_dict(self, name):
        if name not in ARUCO_DICT:
            raise ValueError("Unsupported aruco dictionary: {}".format(name))
        dict_id = ARUCO_DICT[name]
        if hasattr(cv2.aruco, "getPredefinedDictionary"):
            return cv2.aruco.getPredefinedDictionary(dict_id)
        return cv2.aruco.Dictionary_get(dict_id)

    @staticmethod
    def _make_aruco_params():
        if hasattr(cv2.aruco, "DetectorParameters_create"):
            return cv2.aruco.DetectorParameters_create()
        return cv2.aruco.DetectorParameters()

    def _detect_markers(self, gray):
        if self.aruco_detector is not None:
            return self.aruco_detector.detectMarkers(gray)
        return cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

    @staticmethod
    def _to_point32(x, y):
        point = Point32()
        point.x = float(x)
        point.y = float(y)
        point.z = 0.0
        return point

    def _build_detection_msg(self, marker_id, corners_px):
        detection = TagDetection()
        detection.id = int(marker_id)
        detection.dictionary_name = self.aruco_dict_name
        center_px = np.mean(corners_px, axis=0)
        detection.center_px = self._to_point32(center_px[0], center_px[1])
        detection.corners_px = [self._to_point32(pt[0], pt[1]) for pt in corners_px]
        detection.tag_area_px = self._polygon_area_px(corners_px)
        detection.detection_score = self._compute_detection_score(detection.tag_area_px)
        return detection

    def _image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            rospy.logwarn_throttle(5.0, "cv_bridge conversion failed: %s", str(exc))
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detect_markers(gray)

        detections = []
        kept_corners = []
        kept_ids = []
        if ids is not None and len(ids) > 0:
            ids = ids.flatten().tolist()
            for idx, marker_id in enumerate(ids):
                if marker_id not in self.layout_marker_ids:
                    continue

                marker_corners = np.asarray(corners[idx], dtype=np.float32).reshape(4, 2)
                area_px = self._polygon_area_px(marker_corners)
                if area_px < self.min_tag_area_px:
                    continue

                detection = self._build_detection_msg(marker_id, marker_corners)
                detections.append(detection)
                kept_corners.append(corners[idx])
                kept_ids.append(marker_id)

        detections.sort(key=lambda detection: detection.id)

        detection_array = TagDetectionArray()
        detection_array.header = msg.header
        detection_array.image_width = int(frame.shape[1])
        detection_array.image_height = int(frame.shape[0])
        detection_array.detections = detections
        self.detection_pub.publish(detection_array)

        if self.publish_debug_image and self.debug_pub is not None:
            debug_frame = frame.copy()
            if kept_corners and hasattr(cv2.aruco, "drawDetectedMarkers"):
                debug_ids = np.asarray(kept_ids, dtype=np.int32).reshape(-1, 1)
                cv2.aruco.drawDetectedMarkers(debug_frame, kept_corners, debug_ids)
            for detection in detections:
                label = "{} {:.2f}".format(detection.id, detection.detection_score)
                center = (int(round(detection.center_px.x)), int(round(detection.center_px.y)))
                cv2.putText(
                    debug_frame,
                    label,
                    (center[0] + 8, center[1] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    1,
                    cv2.LINE_AA,
                )

            try:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding="bgr8")
                debug_msg.header = msg.header
                self.debug_pub.publish(debug_msg)
            except CvBridgeError:
                rospy.logwarn_throttle(5.0, "Failed to publish paper tag detector debug image.")


if __name__ == "__main__":
    try:
        PaperTagDetector()
        rospy.spin()
    except Exception as exc:
        rospy.logfatal("paper_tag_detector failed: %s", str(exc))
