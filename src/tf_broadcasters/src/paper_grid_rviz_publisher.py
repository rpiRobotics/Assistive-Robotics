#!/usr/bin/env python3

from __future__ import print_function

import json
import os
import threading

import cv2
import numpy as np
import rospy
import tf2_ros
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


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


class PaperGridRvizPublisher(object):
    def __init__(self):
        rospy.init_node("paper_grid_rviz_publisher", anonymous=False)

        self.layout_json_path = os.path.expanduser(rospy.get_param("~layout_json_path", ""))
        if not self.layout_json_path:
            rospy.logfatal("~layout_json_path must be set.")
            raise RuntimeError("Missing layout_json_path")

        self.image_topic_name = rospy.get_param("~image_topic_name", "/nuc/rgb/image_raw")
        self.camera_info_topic_name = rospy.get_param("~camera_info_topic_name", "/nuc/rgb/camera_info")
        self.camera_frame_id = rospy.get_param("~camera_frame_id", "")
        self.target_frame_id = rospy.get_param("~target_frame_id", "")

        self.marker_topic_name = rospy.get_param("~marker_topic_name", "paper_grid_markers")
        self.pose_topic_name = rospy.get_param("~pose_topic_name", "paper_grid_pose")
        self.publish_debug_image = rospy.get_param("~publish_debug_image", True)
        self.debug_image_topic_name = rospy.get_param("~debug_image_topic_name", "paper_grid_debug_image")
        self.depth_image_topic_name = rospy.get_param("~depth_image_topic_name", "")
        self.use_depth_for_3d_points = rospy.get_param("~use_depth_for_3d_points", False)
        self.depth_unit_scale = float(rospy.get_param("~depth_unit_scale", 0.001))  # mm->m for 16UC1
        self.depth_max_age_sec = float(rospy.get_param("~depth_max_age_sec", 0.15))
        self.depth_patch_radius_px = int(rospy.get_param("~depth_patch_radius_px", 2))

        self.min_markers_for_surface = int(
            rospy.get_param("~min_markers_for_surface", rospy.get_param("~min_markers_for_pose", 4))
        )
        self.tf_lookup_timeout_sec = float(rospy.get_param("~tf_lookup_timeout_sec", 0.05))
        self.pnp_reprojection_error = float(rospy.get_param("~pnp_reprojection_error", 3.0))

        self.grid_line_width = float(rospy.get_param("~grid_line_width", 0.003))
        self.outline_line_width = float(rospy.get_param("~outline_line_width", 0.006))
        self.surface_alpha = float(rospy.get_param("~surface_alpha", 0.12))
        self.point_scale = float(rospy.get_param("~point_scale", 0.012))
        self.text_height = float(rospy.get_param("~text_height", 0.03))
        self.text_z_offset = float(rospy.get_param("~text_z_offset", 0.01))
        self.normal_length = float(rospy.get_param("~normal_length", 0.05))
        self.normal_line_width = float(rospy.get_param("~normal_line_width", 0.003))

        self.layout_data = self._load_layout(self.layout_json_path)
        self.grid_rows, self.grid_cols = self._read_grid_shape(self.layout_data)
        self.layout_marker_ids = []
        self.id_to_rowcol = {}
        self.rowcol_to_id = {}
        self.id_to_size_m = {}
        self.default_marker_size_m = float(self.layout_data.get("grid", {}).get("marker_mm", 60.0)) / 1000.0
        self._read_marker_layout(self.layout_data)

        self.boundary_pairs = self._build_boundary_pairs()
        self.surface_triangles = self._build_surface_triangles()
        self.mesh_line_pairs = self._build_mesh_line_pairs()

        self.obj_corners_cache = {}

        layout_dict_name = self.layout_data.get("grid", {}).get("dict", "DICT_4X4_50")
        self.aruco_dict_name = rospy.get_param("~aruco_dict_name", layout_dict_name)
        self.aruco_dict = self._make_aruco_dict(self.aruco_dict_name)
        self.aruco_params = self._make_aruco_params()
        self.aruco_detector = None
        if hasattr(cv2.aruco, "ArucoDetector"):
            self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.pnp_square_flag = cv2.SOLVEPNP_ITERATIVE
        if hasattr(cv2, "SOLVEPNP_IPPE_SQUARE"):
            self.pnp_square_flag = cv2.SOLVEPNP_IPPE_SQUARE

        self.bridge = CvBridge()
        self.marker_pub = rospy.Publisher(self.marker_topic_name, MarkerArray, queue_size=1)
        self.pose_pub = rospy.Publisher(self.pose_topic_name, PoseStamped, queue_size=1)
        self.debug_image_pub = None
        if self.publish_debug_image:
            self.debug_image_pub = rospy.Publisher(self.debug_image_topic_name, Image, queue_size=1)

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.K = None
        self.D = None
        self.camera_info_ready = False
        self.markers_visible = False
        self.data_lock = threading.Lock()
        self.depth_lock = threading.Lock()
        self.latest_depth = None
        self.latest_depth_stamp = None

        self.camera_info_sub = rospy.Subscriber(
            self.camera_info_topic_name, CameraInfo, self._camera_info_cb, queue_size=1
        )
        self.image_sub = rospy.Subscriber(self.image_topic_name, Image, self._image_cb, queue_size=1)
        self.depth_sub = None
        if self.depth_image_topic_name:
            self.depth_sub = rospy.Subscriber(self.depth_image_topic_name, Image, self._depth_cb, queue_size=1)
            rospy.loginfo("Depth subscription enabled: %s", self.depth_image_topic_name)
            if not self.use_depth_for_3d_points:
                rospy.logwarn("Depth topic provided but ~use_depth_for_3d_points is false.")

        rospy.loginfo("paper_grid_rviz_publisher initialized.")
        rospy.loginfo("Layout markers loaded: %d", len(self.layout_marker_ids))
        rospy.loginfo("Grid shape: %d rows x %d cols", self.grid_rows, self.grid_cols)
        rospy.loginfo("Aruco dictionary: %s", self.aruco_dict_name)

    def _load_layout(self, path):
        if not os.path.exists(path):
            raise IOError("Layout json not found: {}".format(path))
        with open(path, "r") as f:
            return json.load(f)

    def _read_grid_shape(self, layout_data):
        grid = layout_data.get("grid", {})
        rows = int(grid.get("rows", 0))
        cols = int(grid.get("cols", 0))
        if rows <= 0 or cols <= 0:
            raise ValueError("Invalid rows/cols in layout json")
        return rows, cols

    def _read_marker_layout(self, layout_data):
        markers = layout_data.get("markers", [])
        if not markers:
            raise ValueError("Layout has no markers")

        for marker in markers:
            marker_id = int(marker["id"])
            row = int(marker["row"])
            col = int(marker["col"])
            size_m = float(marker.get("marker_length_mm", self.default_marker_size_m * 1000.0)) / 1000.0

            if not (0 <= row < self.grid_rows and 0 <= col < self.grid_cols):
                raise ValueError("Marker {} row/col out of bounds".format(marker_id))

            self.layout_marker_ids.append(marker_id)
            self.id_to_rowcol[marker_id] = (row, col)
            self.rowcol_to_id[(row, col)] = marker_id
            self.id_to_size_m[marker_id] = size_m

        self.layout_marker_ids = sorted(self.layout_marker_ids)

    def _build_boundary_pairs(self):
        pairs = []

        # Top row
        for c in range(self.grid_cols - 1):
            pairs.append(((0, c), (0, c + 1)))
        # Right column
        for r in range(self.grid_rows - 1):
            pairs.append(((r, self.grid_cols - 1), (r + 1, self.grid_cols - 1)))
        # Bottom row (reverse)
        for c in range(self.grid_cols - 1, 0, -1):
            pairs.append(((self.grid_rows - 1, c), (self.grid_rows - 1, c - 1)))
        # Left column (reverse)
        for r in range(self.grid_rows - 1, 0, -1):
            pairs.append(((r, 0), (r - 1, 0)))

        id_pairs = []
        for a_rc, b_rc in pairs:
            a_id = self.rowcol_to_id.get(a_rc)
            b_id = self.rowcol_to_id.get(b_rc)
            if a_id is not None and b_id is not None:
                id_pairs.append((a_id, b_id))
        return id_pairs

    def _build_surface_triangles(self):
        triangles = []
        for r in range(self.grid_rows - 1):
            for c in range(self.grid_cols - 1):
                id00 = self.rowcol_to_id.get((r, c))
                id01 = self.rowcol_to_id.get((r, c + 1))
                id10 = self.rowcol_to_id.get((r + 1, c))
                id11 = self.rowcol_to_id.get((r + 1, c + 1))
                if id00 is None or id01 is None or id10 is None or id11 is None:
                    continue
                triangles.append((id00, id01, id11))
                triangles.append((id00, id11, id10))
        return triangles

    def _build_mesh_line_pairs(self):
        pairs = []
        for r in range(self.grid_rows):
            for c in range(self.grid_cols - 1):
                a_id = self.rowcol_to_id.get((r, c))
                b_id = self.rowcol_to_id.get((r, c + 1))
                if a_id is not None and b_id is not None:
                    pairs.append((a_id, b_id))
        for r in range(self.grid_rows - 1):
            for c in range(self.grid_cols):
                a_id = self.rowcol_to_id.get((r, c))
                b_id = self.rowcol_to_id.get((r + 1, c))
                if a_id is not None and b_id is not None:
                    pairs.append((a_id, b_id))
        return pairs

    def _make_aruco_dict(self, name):
        if name not in ARUCO_DICT:
            raise ValueError("Unsupported aruco dictionary: {}".format(name))
        dict_id = ARUCO_DICT[name]
        if hasattr(cv2.aruco, "getPredefinedDictionary"):
            return cv2.aruco.getPredefinedDictionary(dict_id)
        return cv2.aruco.Dictionary_get(dict_id)

    def _make_aruco_params(self):
        if hasattr(cv2.aruco, "DetectorParameters_create"):
            return cv2.aruco.DetectorParameters_create()
        return cv2.aruco.DetectorParameters()

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

    def _detect_markers(self, gray):
        if self.aruco_detector is not None:
            return self.aruco_detector.detectMarkers(gray)
        return cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

    def _marker_object_corners(self, marker_size_m):
        key = round(float(marker_size_m), 6)
        if key in self.obj_corners_cache:
            return self.obj_corners_cache[key]

        half = key * 0.5
        obj = np.asarray(
            [
                [-half, half, 0.0],   # top-left
                [half, half, 0.0],    # top-right
                [half, -half, 0.0],   # bottom-right
                [-half, -half, 0.0],  # bottom-left
            ],
            dtype=np.float64,
        )
        self.obj_corners_cache[key] = obj
        return obj

    def _estimate_marker_center_cam(self, marker_corners_img, marker_size_m, K, D):
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
            return None, None, None

        projected, _ = cv2.projectPoints(obj, rvec, tvec, K, D)
        reproj_error = np.mean(np.linalg.norm(projected.reshape(-1, 2) - marker_corners_img, axis=1))
        if reproj_error > self.pnp_reprojection_error:
            return None, None, None

        center_cam = np.asarray(tvec, dtype=np.float64).reshape(3)
        return center_cam, rvec, tvec

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

        vals = patch.reshape(-1)
        depth_m_vals = []
        for x in vals:
            d = self._depth_value_to_m(float(x))
            if d is not None:
                depth_m_vals.append(d)

        if not depth_m_vals:
            return None
        return float(np.median(np.asarray(depth_m_vals, dtype=np.float64)))

    @staticmethod
    def _pixel_to_cam(K, u, v, z_m):
        fx = K[0, 0]
        fy = K[1, 1]
        cx = K[0, 2]
        cy = K[1, 2]
        x = (u - cx) * z_m / fx
        y = (v - cy) * z_m / fy
        return np.asarray([x, y, z_m], dtype=np.float64)

    def _image_cb(self, msg):
        with self.data_lock:
            if not self.camera_info_ready:
                return
            K = self.K.copy()
            D = self.D.copy()
            camera_frame_id = self.camera_frame_id

        if not camera_frame_id:
            rospy.logwarn_throttle(5.0, "Camera frame is empty; waiting for CameraInfo.")
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            rospy.logwarn_throttle(5.0, "cv_bridge conversion failed: %s", str(exc))
            return

        stamp = msg.header.stamp if msg.header.stamp != rospy.Time(0) else rospy.Time.now()
        target_frame = self.target_frame_id if self.target_frame_id else camera_frame_id
        R_tc, t_tc = self._camera_to_target_transform(camera_frame_id, target_frame, stamp)
        if R_tc is None:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detect_markers(gray)

        detected_points_target = {}
        detected_normals_target = {}
        depth_img = None
        depth_stamp = None
        if self.use_depth_for_3d_points:
            with self.depth_lock:
                if self.latest_depth is not None:
                    depth_img = self.latest_depth.copy()
                    depth_stamp = self.latest_depth_stamp

        if ids is not None and len(ids) > 0:
            ids = ids.flatten().tolist()
            for i, marker_id in enumerate(ids):
                if marker_id not in self.id_to_rowcol:
                    continue
                marker_size_m = self.id_to_size_m.get(marker_id, self.default_marker_size_m)
                marker_corners_img = np.asarray(corners[i], dtype=np.float64).reshape(4, 2)

                center_cam, rvec, tvec = self._estimate_marker_center_cam(
                    marker_corners_img, marker_size_m, K, D
                )
                if center_cam is None:
                    continue

                R_cm, _ = cv2.Rodrigues(rvec)
                normal_cam = R_cm[:, 2]
                if np.linalg.norm(normal_cam) > 1e-8:
                    normal_cam = normal_cam / np.linalg.norm(normal_cam)

                # Optional depth override for better non-planar reconstruction.
                if self.use_depth_for_3d_points and depth_img is not None and depth_stamp is not None:
                    age = abs((stamp - depth_stamp).to_sec())
                    if age <= self.depth_max_age_sec:
                        center_px = np.mean(marker_corners_img, axis=0)
                        depth_m = self._sample_depth_m(depth_img, center_px[0], center_px[1])
                        if depth_m is not None:
                            center_cam = self._pixel_to_cam(K, center_px[0], center_px[1], depth_m)

                center_target = np.matmul(R_tc, center_cam.reshape(3, 1)) + t_tc
                detected_points_target[marker_id] = center_target.reshape(3)
                normal_target = np.matmul(R_tc, normal_cam.reshape(3, 1)).reshape(3)
                if np.linalg.norm(normal_target) > 1e-8:
                    normal_target = normal_target / np.linalg.norm(normal_target)
                detected_normals_target[marker_id] = normal_target

                if self.publish_debug_image and hasattr(cv2.aruco, "drawDetectedMarkers"):
                    cv2.aruco.drawDetectedMarkers(frame, [corners[i]], np.asarray([[marker_id]], dtype=np.int32))
                    if hasattr(cv2.aruco, "drawAxis"):
                        cv2.aruco.drawAxis(frame, K, D, rvec, tvec, marker_size_m * 0.5)

        if self.publish_debug_image and self.debug_image_pub is not None:
            self._publish_debug_image(frame, stamp)

        if len(detected_points_target) < self.min_markers_for_surface:
            self._clear_markers_if_needed(stamp, target_frame)
            rospy.logwarn_throttle(
                2.0,
                "Detected %d layout markers, need at least %d for deformable surface.",
                len(detected_points_target),
                self.min_markers_for_surface,
            )
            return

        self._publish_pose_from_points(stamp, target_frame, detected_points_target)
        self._publish_deformed_markers(stamp, target_frame, detected_points_target, detected_normals_target)
        self.markers_visible = True

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
        m = R
        trace = float(m[0, 0] + m[1, 1] + m[2, 2])
        if trace > 0.0:
            s = np.sqrt(trace + 1.0) * 2.0
            qw = 0.25 * s
            qx = (m[2, 1] - m[1, 2]) / s
            qy = (m[0, 2] - m[2, 0]) / s
            qz = (m[1, 0] - m[0, 1]) / s
        elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
            s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
            qw = (m[2, 1] - m[1, 2]) / s
            qx = 0.25 * s
            qy = (m[0, 1] + m[1, 0]) / s
            qz = (m[0, 2] + m[2, 0]) / s
        elif m[1, 1] > m[2, 2]:
            s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
            qw = (m[0, 2] - m[2, 0]) / s
            qx = (m[0, 1] + m[1, 0]) / s
            qy = 0.25 * s
            qz = (m[1, 2] + m[2, 1]) / s
        else:
            s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
            qw = (m[1, 0] - m[0, 1]) / s
            qx = (m[0, 2] + m[2, 0]) / s
            qy = (m[1, 2] + m[2, 1]) / s
            qz = 0.25 * s
        return qx, qy, qz, qw

    @staticmethod
    def _to_point(x, y, z):
        p = Point()
        p.x = float(x)
        p.y = float(y)
        p.z = float(z)
        return p

    def _publish_pose_from_points(self, stamp, frame_id, points_by_id):
        if len(points_by_id) < 3:
            return

        pts = np.asarray(list(points_by_id.values()), dtype=np.float64)
        centroid = np.mean(pts, axis=0)
        centered = pts - centroid

        try:
            _, _, vh = np.linalg.svd(centered, full_matrices=False)
        except np.linalg.LinAlgError:
            return

        x_axis = vh[0, :]
        normal = vh[2, :]

        if np.linalg.norm(x_axis) < 1e-8 or np.linalg.norm(normal) < 1e-8:
            return

        x_axis = x_axis / np.linalg.norm(x_axis)
        normal = normal / np.linalg.norm(normal)
        y_axis = np.cross(normal, x_axis)
        if np.linalg.norm(y_axis) < 1e-8:
            return
        y_axis = y_axis / np.linalg.norm(y_axis)
        x_axis = np.cross(y_axis, normal)
        x_axis = x_axis / np.linalg.norm(x_axis)

        R = np.column_stack((x_axis, y_axis, normal))
        qx, qy, qz, qw = self._rot_to_quat(R)

        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.pose.position.x = float(centroid[0])
        msg.pose.position.y = float(centroid[1])
        msg.pose.position.z = float(centroid[2])
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.pose_pub.publish(msg)

    def _publish_deformed_markers(self, stamp, frame_id, points_by_id, normals_by_id):
        msg = MarkerArray()

        # Deformed surface from detected tag points.
        surface = Marker()
        surface.header.stamp = stamp
        surface.header.frame_id = frame_id
        surface.ns = "paper_surface"
        surface.id = 0
        surface.type = Marker.TRIANGLE_LIST
        surface.action = Marker.ADD
        surface.pose.orientation.w = 1.0
        surface.scale.x = 1.0
        surface.scale.y = 1.0
        surface.scale.z = 1.0
        surface.color.r = 0.1
        surface.color.g = 0.5
        surface.color.b = 1.0
        surface.color.a = self.surface_alpha

        for tri in self.surface_triangles:
            if tri[0] in points_by_id and tri[1] in points_by_id and tri[2] in points_by_id:
                p0 = points_by_id[tri[0]]
                p1 = points_by_id[tri[1]]
                p2 = points_by_id[tri[2]]
                surface.points.append(self._to_point(*p0))
                surface.points.append(self._to_point(*p1))
                surface.points.append(self._to_point(*p2))
        msg.markers.append(surface)

        # Mesh lines between neighboring tags.
        grid = Marker()
        grid.header.stamp = stamp
        grid.header.frame_id = frame_id
        grid.ns = "aruco_grid"
        grid.id = 1
        grid.type = Marker.LINE_LIST
        grid.action = Marker.ADD
        grid.pose.orientation.w = 1.0
        grid.scale.x = self.grid_line_width
        grid.color.r = 0.0
        grid.color.g = 1.0
        grid.color.b = 0.2
        grid.color.a = 0.95
        for a_id, b_id in self.mesh_line_pairs:
            if a_id in points_by_id and b_id in points_by_id:
                grid.points.append(self._to_point(*points_by_id[a_id]))
                grid.points.append(self._to_point(*points_by_id[b_id]))
        msg.markers.append(grid)

        # Boundary lines.
        outline = Marker()
        outline.header.stamp = stamp
        outline.header.frame_id = frame_id
        outline.ns = "paper_outline"
        outline.id = 2
        outline.type = Marker.LINE_LIST
        outline.action = Marker.ADD
        outline.pose.orientation.w = 1.0
        outline.scale.x = self.outline_line_width
        outline.color.r = 1.0
        outline.color.g = 0.2
        outline.color.b = 0.2
        outline.color.a = 1.0
        for a_id, b_id in self.boundary_pairs:
            if a_id in points_by_id and b_id in points_by_id:
                outline.points.append(self._to_point(*points_by_id[a_id]))
                outline.points.append(self._to_point(*points_by_id[b_id]))
        msg.markers.append(outline)

        # Tag points.
        points_marker = Marker()
        points_marker.header.stamp = stamp
        points_marker.header.frame_id = frame_id
        points_marker.ns = "paper_points"
        points_marker.id = 3
        points_marker.type = Marker.SPHERE_LIST
        points_marker.action = Marker.ADD
        points_marker.pose.orientation.w = 1.0
        points_marker.scale.x = self.point_scale
        points_marker.scale.y = self.point_scale
        points_marker.scale.z = self.point_scale
        points_marker.color.r = 1.0
        points_marker.color.g = 0.8
        points_marker.color.b = 0.0
        points_marker.color.a = 1.0
        for marker_id in sorted(points_by_id.keys()):
            points_marker.points.append(self._to_point(*points_by_id[marker_id]))
        msg.markers.append(points_marker)

        # Tag IDs on top of detected points.
        for marker_id in sorted(points_by_id.keys()):
            p = points_by_id[marker_id]
            text = Marker()
            text.header.stamp = stamp
            text.header.frame_id = frame_id
            text.ns = "aruco_ids"
            text.id = 1000 + int(marker_id)
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = float(p[0])
            text.pose.position.y = float(p[1])
            text.pose.position.z = float(p[2] + self.text_z_offset)
            text.pose.orientation.w = 1.0
            text.scale.z = self.text_height
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0
            text.text = str(marker_id)
            msg.markers.append(text)

        # Per-tag surface normals.
        normals = Marker()
        normals.header.stamp = stamp
        normals.header.frame_id = frame_id
        normals.ns = "paper_normals"
        normals.id = 4
        normals.type = Marker.LINE_LIST
        normals.action = Marker.ADD
        normals.pose.orientation.w = 1.0
        normals.scale.x = self.normal_line_width
        normals.color.r = 1.0
        normals.color.g = 0.0
        normals.color.b = 1.0
        normals.color.a = 1.0
        for marker_id in sorted(points_by_id.keys()):
            if marker_id not in normals_by_id:
                continue
            p = points_by_id[marker_id]
            n = normals_by_id[marker_id]
            p2 = p + n * self.normal_length
            normals.points.append(self._to_point(*p))
            normals.points.append(self._to_point(*p2))
        msg.markers.append(normals)

        self.marker_pub.publish(msg)

    def _clear_markers_if_needed(self, stamp, frame_id):
        if not self.markers_visible:
            return
        clear = MarkerArray()
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = frame_id
        marker.action = Marker.DELETEALL
        clear.markers.append(marker)
        self.marker_pub.publish(clear)
        self.markers_visible = False

    def _publish_debug_image(self, cv_img, stamp):
        try:
            msg = self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
            msg.header.stamp = stamp
            self.debug_image_pub.publish(msg)
        except CvBridgeError:
            pass


if __name__ == "__main__":
    try:
        PaperGridRvizPublisher()
        rospy.spin()
    except Exception as exc:
        rospy.logfatal("paper_grid_rviz_publisher failed: %s", str(exc))
