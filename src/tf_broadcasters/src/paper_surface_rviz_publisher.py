#!/usr/bin/env python3

from __future__ import print_function

import json
import os

import numpy as np
import rospy
from assistive_msgs.msg import TagPoseArray
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


class PaperSurfaceRvizPublisher(object):
    def __init__(self):
        rospy.init_node("paper_surface_rviz_publisher", anonymous=False)

        self.layout_json_path = os.path.expanduser(rospy.get_param("~layout_json_path", ""))
        if not self.layout_json_path:
            rospy.logfatal("~layout_json_path must be set.")
            raise RuntimeError("Missing layout_json_path")

        self.pose_topic_name = rospy.get_param("~pose_topic_name", "paper_surface_state")
        self.marker_topic_name = rospy.get_param("~marker_topic_name", "paper_surface_markers")
        self.grid_line_width = float(rospy.get_param("~grid_line_width", 0.003))
        self.outline_line_width = float(rospy.get_param("~outline_line_width", 0.006))
        self.surface_alpha = float(rospy.get_param("~surface_alpha", 0.12))
        self.predicted_surface_alpha_scale = float(rospy.get_param("~predicted_surface_alpha_scale", 0.55))
        self.point_scale = float(rospy.get_param("~point_scale", 0.012))
        self.predicted_point_scale = float(rospy.get_param("~predicted_point_scale", 0.018))
        self.uncertainty_scale_gain = float(rospy.get_param("~uncertainty_scale_gain", 0.12))
        self.text_height = float(rospy.get_param("~text_height", 0.03))
        self.text_z_offset = float(rospy.get_param("~text_z_offset", 0.01))
        self.normal_length = float(rospy.get_param("~normal_length", 0.05))
        self.normal_line_width = float(rospy.get_param("~normal_line_width", 0.003))

        layout_data = self._load_layout(self.layout_json_path)
        self.grid_rows, self.grid_cols = self._read_grid_shape(layout_data)
        self.rowcol_to_id = self._read_marker_layout(layout_data)
        self.boundary_pairs = self._build_boundary_pairs()
        self.surface_triangles = self._build_surface_triangles()
        self.mesh_line_pairs = self._build_mesh_line_pairs()

        self.marker_pub = rospy.Publisher(self.marker_topic_name, MarkerArray, queue_size=1)
        self.pose_sub = rospy.Subscriber(self.pose_topic_name, TagPoseArray, self._pose_cb, queue_size=1)
        self.markers_visible = False

        rospy.loginfo("paper_surface_rviz_publisher initialized.")
        rospy.loginfo("Publishing markers on: %s", self.marker_topic_name)

    def _load_layout(self, path):
        if not os.path.exists(path):
            raise IOError("Layout json not found: {}".format(path))
        with open(path, "r") as f:
            return json.load(f)

    @staticmethod
    def _read_grid_shape(layout_data):
        grid = layout_data.get("grid", {})
        rows = int(grid.get("rows", 0))
        cols = int(grid.get("cols", 0))
        if rows <= 0 or cols <= 0:
            raise ValueError("Invalid rows/cols in layout json")
        return rows, cols

    def _read_marker_layout(self, layout_data):
        rowcol_to_id = {}
        for marker in layout_data.get("markers", []):
            rowcol_to_id[(int(marker["row"]), int(marker["col"]))] = int(marker["id"])
        if not rowcol_to_id:
            raise ValueError("Layout has no markers")
        return rowcol_to_id

    def _build_boundary_pairs(self):
        pairs = []
        for c in range(self.grid_cols - 1):
            pairs.append(((0, c), (0, c + 1)))
        for r in range(self.grid_rows - 1):
            pairs.append(((r, self.grid_cols - 1), (r + 1, self.grid_cols - 1)))
        for c in range(self.grid_cols - 1, 0, -1):
            pairs.append(((self.grid_rows - 1, c), (self.grid_rows - 1, c - 1)))
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

    @staticmethod
    def _to_point(x, y, z):
        point = Point()
        point.x = float(x)
        point.y = float(y)
        point.z = float(z)
        return point

    def _pose_cb(self, msg):
        if not msg.poses:
            self._clear_markers_if_needed(msg.header.stamp, msg.header.frame_id)
            return

        points_by_id = {}
        normals_by_id = {}
        observed_by_id = {}
        position_var_by_id = {}
        for pose in msg.poses:
            marker_id = int(pose.id)
            points_by_id[marker_id] = np.asarray(
                [
                    pose.pose.pose.position.x,
                    pose.pose.pose.position.y,
                    pose.pose.pose.position.z,
                ],
                dtype=np.float64,
            )
            normals_by_id[marker_id] = np.asarray([pose.normal.x, pose.normal.y, pose.normal.z], dtype=np.float64)
            observed_by_id[marker_id] = bool(pose.observed)
            covariance = np.asarray(pose.pose.covariance, dtype=np.float64)
            position_var_by_id[marker_id] = float(np.mean(covariance[[0, 7, 14]])) if covariance.size >= 15 else 0.0

        self._publish_markers(
            msg.header.stamp,
            msg.header.frame_id,
            points_by_id,
            normals_by_id,
            observed_by_id,
            position_var_by_id,
        )
        self.markers_visible = True

    def _publish_markers(self, stamp, frame_id, points_by_id, normals_by_id, observed_by_id, position_var_by_id):
        msg = MarkerArray()
        total_count = max(1, len(points_by_id))
        observed_count = sum(1 for observed in observed_by_id.values() if observed)
        observed_ratio = float(observed_count) / float(total_count)

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
        surface.color.a = self.surface_alpha * (
            self.predicted_surface_alpha_scale + (1.0 - self.predicted_surface_alpha_scale) * observed_ratio
        )
        for tri in self.surface_triangles:
            if tri[0] in points_by_id and tri[1] in points_by_id and tri[2] in points_by_id:
                surface.points.append(self._to_point(*points_by_id[tri[0]]))
                surface.points.append(self._to_point(*points_by_id[tri[1]]))
                surface.points.append(self._to_point(*points_by_id[tri[2]]))
        msg.markers.append(surface)

        observed_grid = Marker()
        observed_grid.header.stamp = stamp
        observed_grid.header.frame_id = frame_id
        observed_grid.ns = "aruco_grid_observed"
        observed_grid.id = 1
        observed_grid.type = Marker.LINE_LIST
        observed_grid.action = Marker.ADD
        observed_grid.pose.orientation.w = 1.0
        observed_grid.scale.x = self.grid_line_width
        observed_grid.color.r = 0.0
        observed_grid.color.g = 1.0
        observed_grid.color.b = 0.2
        observed_grid.color.a = 0.95

        predicted_grid = Marker()
        predicted_grid.header.stamp = stamp
        predicted_grid.header.frame_id = frame_id
        predicted_grid.ns = "aruco_grid_predicted"
        predicted_grid.id = 2
        predicted_grid.type = Marker.LINE_LIST
        predicted_grid.action = Marker.ADD
        predicted_grid.pose.orientation.w = 1.0
        predicted_grid.scale.x = self.grid_line_width
        predicted_grid.color.r = 1.0
        predicted_grid.color.g = 0.7
        predicted_grid.color.b = 0.0
        predicted_grid.color.a = 0.7

        for marker_a, marker_b in self.mesh_line_pairs:
            if marker_a not in points_by_id or marker_b not in points_by_id:
                continue
            line_marker = observed_grid if observed_by_id.get(marker_a, False) and observed_by_id.get(marker_b, False) else predicted_grid
            line_marker.points.append(self._to_point(*points_by_id[marker_a]))
            line_marker.points.append(self._to_point(*points_by_id[marker_b]))
        msg.markers.append(observed_grid)
        msg.markers.append(predicted_grid)

        outline = Marker()
        outline.header.stamp = stamp
        outline.header.frame_id = frame_id
        outline.ns = "paper_outline"
        outline.id = 3
        outline.type = Marker.LINE_LIST
        outline.action = Marker.ADD
        outline.pose.orientation.w = 1.0
        outline.scale.x = self.outline_line_width
        outline.color.r = 1.0
        outline.color.g = 0.2
        outline.color.b = 0.2
        outline.color.a = 1.0
        for marker_a, marker_b in self.boundary_pairs:
            if marker_a in points_by_id and marker_b in points_by_id:
                outline.points.append(self._to_point(*points_by_id[marker_a]))
                outline.points.append(self._to_point(*points_by_id[marker_b]))
        msg.markers.append(outline)

        observed_points = Marker()
        observed_points.header.stamp = stamp
        observed_points.header.frame_id = frame_id
        observed_points.ns = "paper_points_observed"
        observed_points.id = 4
        observed_points.type = Marker.SPHERE_LIST
        observed_points.action = Marker.ADD
        observed_points.pose.orientation.w = 1.0
        observed_points.scale.x = self.point_scale
        observed_points.scale.y = self.point_scale
        observed_points.scale.z = self.point_scale
        observed_points.color.r = 1.0
        observed_points.color.g = 0.8
        observed_points.color.b = 0.0
        observed_points.color.a = 1.0
        for marker_id, point in sorted(points_by_id.items()):
            if observed_by_id.get(marker_id, False):
                observed_points.points.append(self._to_point(*point))
        msg.markers.append(observed_points)

        observed_normals = Marker()
        observed_normals.header.stamp = stamp
        observed_normals.header.frame_id = frame_id
        observed_normals.ns = "paper_normals_observed"
        observed_normals.id = 5
        observed_normals.type = Marker.LINE_LIST
        observed_normals.action = Marker.ADD
        observed_normals.pose.orientation.w = 1.0
        observed_normals.scale.x = self.normal_line_width
        observed_normals.color.r = 0.0
        observed_normals.color.g = 0.8
        observed_normals.color.b = 1.0
        observed_normals.color.a = 1.0

        predicted_normals = Marker()
        predicted_normals.header.stamp = stamp
        predicted_normals.header.frame_id = frame_id
        predicted_normals.ns = "paper_normals_predicted"
        predicted_normals.id = 6
        predicted_normals.type = Marker.LINE_LIST
        predicted_normals.action = Marker.ADD
        predicted_normals.pose.orientation.w = 1.0
        predicted_normals.scale.x = self.normal_line_width
        predicted_normals.color.r = 1.0
        predicted_normals.color.g = 0.0
        predicted_normals.color.b = 1.0
        predicted_normals.color.a = 0.7

        for marker_id, point in sorted(points_by_id.items()):
            normal = normals_by_id.get(marker_id)
            if normal is None:
                continue
            endpoint = point + normal * self.normal_length
            line_marker = observed_normals if observed_by_id.get(marker_id, False) else predicted_normals
            line_marker.points.append(self._to_point(*point))
            line_marker.points.append(self._to_point(*endpoint))
        msg.markers.append(observed_normals)
        msg.markers.append(predicted_normals)

        marker_id_offset = 1000
        for marker_id, point in sorted(points_by_id.items()):
            if not observed_by_id.get(marker_id, False):
                variance = max(0.0, position_var_by_id.get(marker_id, 0.0))
                predicted_scale = self.predicted_point_scale + self.uncertainty_scale_gain * np.sqrt(variance)
                predicted_marker = Marker()
                predicted_marker.header.stamp = stamp
                predicted_marker.header.frame_id = frame_id
                predicted_marker.ns = "paper_points_predicted"
                predicted_marker.id = marker_id_offset + marker_id
                predicted_marker.type = Marker.SPHERE
                predicted_marker.action = Marker.ADD
                predicted_marker.pose.position.x = float(point[0])
                predicted_marker.pose.position.y = float(point[1])
                predicted_marker.pose.position.z = float(point[2])
                predicted_marker.pose.orientation.w = 1.0
                predicted_marker.scale.x = predicted_scale
                predicted_marker.scale.y = predicted_scale
                predicted_marker.scale.z = predicted_scale
                predicted_marker.color.r = 1.0
                predicted_marker.color.g = 0.45
                predicted_marker.color.b = 0.15
                predicted_marker.color.a = 0.45
                msg.markers.append(predicted_marker)

            text = Marker()
            text.header.stamp = stamp
            text.header.frame_id = frame_id
            text.ns = "aruco_ids"
            text.id = 2000 + marker_id
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = float(point[0])
            text.pose.position.y = float(point[1])
            text.pose.position.z = float(point[2] + self.text_z_offset)
            text.pose.orientation.w = 1.0
            text.scale.z = self.text_height
            if observed_by_id.get(marker_id, False):
                text.color.r = 1.0
                text.color.g = 1.0
                text.color.b = 1.0
                text.color.a = 1.0
                text.text = str(marker_id)
            else:
                text.color.r = 1.0
                text.color.g = 0.65
                text.color.b = 0.0
                text.color.a = 0.9
                text.text = "{}*".format(marker_id)
            msg.markers.append(text)

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


if __name__ == "__main__":
    try:
        PaperSurfaceRvizPublisher()
        rospy.spin()
    except Exception as exc:
        rospy.logfatal("paper_surface_rviz_publisher failed: %s", str(exc))
