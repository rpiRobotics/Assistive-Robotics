#!/usr/bin/env python3

"""
Author: Burak Aksoy
Node: collision_avoidance_2d
Description:
    TODO

Parameters:
    - TODO
Subscribes to:
    - /$(robot)/cmd_vel_in (eg. geometry_msgs::Twist)
    - /tf
Publishes to:
    - /$(robot)/cmd_vel_out (eg. /j2n6s300_driver/in/cartesian_velocity (kinova_msgs::CartesianVelocity))
    - TODO for debuggers visualizations
"""

import rospy
import tf2_ros

import geometry_msgs.msg # for Twist and other visualization msgs

from obstacle_detector.msg import Obstacles

from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse

import shapely
import shapely.geometry
from shapely.ops import nearest_points
from shapely.ops import triangulate

import numpy as np
import math

class CollisionAvoidance2D():
    def __init__(self):

        rospy.init_node('collision_avoidance_2d', anonymous=True)

        # Published topic names 
        self.out_cmd_vel_topic_name = rospy.get_param("~out_cmd_vel_topic_name")
        # Publishers
        self.pub_cmd_vel = rospy.Publisher(self.out_cmd_vel_topic_name, geometry_msgs.msg.Twist, queue_size=1)

        # To store the velocities
        self.Vx = 0.0
        self.Vy = 0.0
        self.Vz = 0.0
        self.Wx = 0.0
        self.Wy = 0.0
        self.Wz = 0.0

        # To store the velocities for filtering
        self.Vx_modified = 0.0
        self.Vy_modified = 0.0
        self.Vz_modified = 0.0
        self.Wx_modified = 0.0
        self.Wy_modified = 0.0
        self.Wz_modified = 0.0

        # To store the laser obstacle readings
        self.laser_obstacles_msg = None

        # Subscribed topic names
        self.in_cmd_vel_topic_name = rospy.get_param("~in_cmd_vel_topic_name")
        self.laser_obstacles_topic_name = rospy.get_param("~laser_obstacles_topic_name")

        # Subscribers
        self.sub_cmd_vel = rospy.Subscriber(self.in_cmd_vel_topic_name, geometry_msgs.msg.Twist, self.cmd_vel_callback, queue_size=1)
        self.sub_laser_obstacles = rospy.Subscriber(self.laser_obstacles_topic_name, Obstacles, self.laser_obstacles_callback, queue_size=1)

        # other parameters
        self.num_of_robots = rospy.get_param("~number_of_robots", 2)
        
        self.tf_world_frame_id = rospy.get_param("~tf_world_frame_id")
        self.tf_mobile_base_frame_id = rospy.get_param("~tf_mobile_base_frame_id")
        
        self.obs_dist_thres = rospy.get_param("~obs_dist_thres")
        self.obs_dist_hard_thres = rospy.get_param("~obs_dist_hard_thres")

        self.all_tf_mobile_base_frame_ids = rospy.get_param("~all_tf_mobile_base_frame_ids")
        self.all_mobile_base_frame_coords = rospy.get_param("~all_mobile_base_frame_coords")

        self.index = self.all_tf_mobile_base_frame_ids.index(self.tf_mobile_base_frame_id)
        
        self.workspace_polygon_coords = rospy.get_param("~workspace_polygon_coords")

        # TF2 listener
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        self.TFs = []
        for i in range(self.num_of_robots):
            self.TFs.append(None)

        self.TFs_are_ready = False
        
        # Debug and Visualizer publishers
        self.viz_mobile_base_polygon_topic_name_prefix = rospy.get_param("~viz_mobile_base_polygon_topic_name_prefix")
        self.viz_mobile_base_obs_dist_thres_topic_name = rospy.get_param("~viz_mobile_base_obs_dist_thres_topic_name")
        self.viz_mobile_base_obs_dist_hard_thres_topic_name = rospy.get_param("~viz_mobile_base_obs_dist_hard_thres_topic_name")
        self.viz_workspace_polygon_topic_name = rospy.get_param("~viz_workspace_polygon_topic_name")

        # create each specified robot's polygon publishers
        self.pubs_viz_mobile_base_polygon = []
        for i in range(self.num_of_robots):
            topic_name = self.viz_mobile_base_polygon_topic_name_prefix + self.all_tf_mobile_base_frame_ids[i]
            publisher = rospy.Publisher(topic_name, geometry_msgs.msg.PolygonStamped, queue_size=2)
            self.pubs_viz_mobile_base_polygon.append(publisher)

        self.pub_viz_mobile_base_obs_dist_thres = rospy.Publisher(self.viz_mobile_base_obs_dist_thres_topic_name, geometry_msgs.msg.PolygonStamped, queue_size=2)
        self.pub_viz_mobile_base_obs_dist_hard_thres = rospy.Publisher(self.viz_mobile_base_obs_dist_hard_thres_topic_name, geometry_msgs.msg.PolygonStamped, queue_size=2)
        self.pub_viz_workspace_polygon = rospy.Publisher(self.viz_workspace_polygon_topic_name, geometry_msgs.msg.PolygonStamped, queue_size=2)

        # Shapely polygon objects allocations
        self.workspace_polygon = None
        self.mobile_base_polygons = []
        for i in range(self.num_of_robots):
            self.mobile_base_polygons.append(None)
        self.obs_dist_thres_polygon = None
        self.obs_dist_hard_thres_polygon = None
        
        self.enable_collision_avoidance = rospy.get_param("~enable_collision_avoidance", True) 
        self.toggle_collision_avoidance_service_name = rospy.get_param("~toggle_collision_avoidance_service_name") 

        self.enable_collision_avoidance_dynamic = rospy.get_param("~enable_collision_avoidance_dynamic", True) 

        # Service to toggle the collision avoidance (enable/disable)
        self.srv_toggle_collision_avoidance = rospy.Service(self.toggle_collision_avoidance_service_name, SetBool, self.srv_toggle_collision_avoidance_cb)

        # Publish rate debuggers/visualizers
        self.viz_out_rate = rospy.get_param("~viz_out_rate", 100.0) 

        self.laser_obstacles_expected_rate = rospy.get_param("~laser_obstacles_expected_rate",40.0) 

        self.line_obstacle_buffer_distance = rospy.get_param("~line_obstacle_buffer_distance", 0.025)

        # Set a timeout to wait for incoming msgs. If there is no incoming cmd msg more than this timeout
        # the node will publish 0 velocities for safety.
        self.cmd_wait_timeout = 3.0 * (1.00/self.viz_out_rate)
        self.time_last_cmd_vel = 0.0

        self.obstacles_wait_timeout = 3.0 * (1.00/self.laser_obstacles_expected_rate)
        self.time_last_laser_obstacles = 0.0

        # Start publishing
        rospy.Timer(rospy.Duration(1.00/self.viz_out_rate), self.run)

    def run(self, event=None):
        if self.enable_collision_avoidance:
            self.TFs_are_ready = self.get_TFs()

            if self.TFs_are_ready:
                # Calculate shapely polygons and publish the visualizers
                self.publish_visualizers()

                # Calculate the obstacles with shapely
                collision_polygons, collision_polygons_hard = self.calculate_obstacles()
                collision_polygons_dynamic, collision_polygons_dynamic_hard = self.calculate_obstacles_dynamic()

                # Calculate the safer cmd_vel
                self.avoid_obstacles(collision_polygons, collision_polygons_hard, collision_polygons_dynamic, collision_polygons_dynamic_hard)
                
                # Publish the safer cmd_vel
                self.publishVelCmd()
                
            else:
                # Publish zero velocities ?
                pass
        else:
            # directly publish the input velocity as the output
            self.Vx_modified = self.Vx
            self.Vy_modified = self.Vy
            self.Vz_modified = self.Vz
            self.Wx_modified = self.Wx
            self.Wy_modified = self.Wy
            self.Wz_modified = self.Wz

            self.publishVelCmd()

        # Check for timeout for incoming msgs
        if (rospy.Time.now().to_sec() - self.time_last_cmd_vel > self.cmd_wait_timeout):
            # If the timeouts, reset the incoming vel cmd msg values to zero.
            self.Vx = 0.0
            self.Vy = 0.0
            self.Vz = 0.0
            self.Wx = 0.0
            self.Wy = 0.0
            self.Wz = 0.0
        
        if (rospy.Time.now().to_sec() - self.time_last_laser_obstacles > self.obstacles_wait_timeout):
            # If the timeouts, reset the incoming laser obstacle msg values to none.
            self.laser_obstacles_msg = None

    
    def get_TFs(self):
        for i in range(self.num_of_robots):
            try:
                # Find the transforms between the robots and the world
                if i == self.index: 
                    tf_ = self.tfBuffer.lookup_transform(self.tf_mobile_base_frame_id, self.tf_world_frame_id,  rospy.Time()) # in the robot frame
                else:
                    tf_ = self.tfBuffer.lookup_transform(self.tf_mobile_base_frame_id, self.all_tf_mobile_base_frame_ids[i],  rospy.Time()) # in the robot frame
                self.TFs[i] = tf_


            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                # Put a warning which says that the transformation could not found
                msg = 'Collision Avoidance 2D: Could not find the transformation from %s to (%s or %s)' \
                        % (self.tf_mobile_base_frame_id, self.tf_world_frame_id, self.all_tf_mobile_base_frame_ids[i])

                # rospy.logwarn(msg)
                rospy.logwarn_throttle(20.0, msg + " (throttled to 20.0s)")

                # reset the saved TFs to None
                # rospy.logwarn('Collision Avoidance 2D: Resetting the saved TF list')
                rospy.logwarn_throttle(20.0, "Collision Avoidance 2D: Resetting the saved TF list (throttled to 20.0s)")
                
                for i in range(self.num_of_robots):
                    self.TFs[i] = None
                return False
        return True

    def publish_visualizers(self):
        # Create the robot's workspace_polygon visualizer
        self.create_workspace_polygon()

        # Create all robots' base frame visualizers
        self.create_mobile_base_frame_polygons()

        # Create the robot's obs_dist_thres visualizer
        self.create_obs_dist_thres()
        
        # Create the robot's obs_dist_thres visualizer
        self.create_obs_dist_hard_thres()

    def create_workspace_polygon(self):
        poly_msg = geometry_msgs.msg.PolygonStamped()

        poly_msg.header.stamp = rospy.Time.now()
        poly_msg.header.frame_id = self.tf_world_frame_id

        poly_msg.polygon.points = [geometry_msgs.msg.Point32(x=corner[0], y=corner[1]) for corner in self.workspace_polygon_coords] 

        self.pub_viz_workspace_polygon.publish(poly_msg)

        r = shapely.geometry.LinearRing(self.workspace_polygon_coords)
        r = shapely.geometry.Polygon(r)
        (x,y,theta) = self.get_2d_pose_from_tf(self.TFs[self.index]) # TF from robot to world
        # Rotate & Translate the polygon to its pose in world frame
        r = shapely.affinity.rotate(r, theta, origin=(0.,0.), use_radians=True)
        r = shapely.affinity.translate(r, xoff = x, yoff = y)
        # rospy.logwarn("Collision Avoidance 2D: workspace_polygon corners xy wrt robot:\n" + '\n'.join(map(str, list(r.exterior.coords))) )
        self.workspace_polygon = r

    def create_mobile_base_frame_polygons(self):
        for i in range(self.num_of_robots):
            poly_msg = geometry_msgs.msg.PolygonStamped()

            poly_msg.header.stamp = rospy.Time.now()
            poly_msg.header.frame_id = self.all_tf_mobile_base_frame_ids[i]

            poly_msg.polygon.points = [geometry_msgs.msg.Point32(x=corner[0], y=corner[1]) for corner in self.all_mobile_base_frame_coords[i]] 

            self.pubs_viz_mobile_base_polygon[i].publish(poly_msg)

            r = shapely.geometry.LinearRing(self.all_mobile_base_frame_coords[i])
            r = shapely.geometry.Polygon(r)

            if i != self.index:
                (x,y,theta) = self.get_2d_pose_from_tf(self.TFs[i])
                # Rotate & Translate the polygon to its pose in robot's frame
                r = shapely.affinity.rotate(r, theta, origin=(0.,0.), use_radians=True)
                r = shapely.affinity.translate(r, xoff = x, yoff = y)
            # rospy.logwarn("Collision Avoidance 2D: mobile_base_frame_polygons[" + str(i)+  "] corners xy:\n" + '\n'.join(map(str, list(r.exterior.coords))) )
            self.mobile_base_polygons[i] = r
            
    def create_obs_dist_thres(self):
        poly_msg = geometry_msgs.msg.PolygonStamped()

        poly_msg.header.stamp = rospy.Time.now()
        poly_msg.header.frame_id = self.tf_mobile_base_frame_id

        r = shapely.geometry.LinearRing(self.all_mobile_base_frame_coords[self.index])
        s = shapely.geometry.Polygon(r)
        r = shapely.geometry.Polygon(s.buffer(self.obs_dist_thres).exterior, [r])
        Xs = list(r.exterior.coords.xy[0])
        Ys = list(r.exterior.coords.xy[1])
        
        poly_msg.polygon.points = [geometry_msgs.msg.Point32(x=Xs[i], y=Ys[i]) for i in range(len(Xs)-1)] # do not include the last point since it is the same with the forst point
        self.pub_viz_mobile_base_obs_dist_thres.publish(poly_msg)

        # (x,y,theta) = self.get_2d_pose_from_tf(self.TFs[self.index])
        # # Rotate & Translate the polygon to its pose in world frame
        # r = shapely.affinity.rotate(r, theta, origin=(0.,0.), use_radians=True)
        # r = shapely.affinity.translate(r, xoff = x, yoff = y)
        # rospy.logwarn("Collision Avoidance 2D: obs_dist_thres_polygon corners xy:\n" + '\n'.join(map(str, list(r.exterior.coords))) )
        self.obs_dist_thres_polygon = r

    def create_obs_dist_hard_thres(self):
        poly_msg = geometry_msgs.msg.PolygonStamped()

        poly_msg.header.stamp = rospy.Time.now()
        poly_msg.header.frame_id = self.tf_mobile_base_frame_id

        r = shapely.geometry.LinearRing(self.all_mobile_base_frame_coords[self.index])
        s = shapely.geometry.Polygon(r)
        r = shapely.geometry.Polygon(s.buffer(self.obs_dist_hard_thres).exterior, [r])
        Xs = list(r.exterior.coords.xy[0])
        Ys = list(r.exterior.coords.xy[1])
        
        poly_msg.polygon.points = [geometry_msgs.msg.Point32(x=Xs[i], y=Ys[i]) for i in range(len(Xs)-1)] # do not include the last point since it is the same with the forst point
        self.pub_viz_mobile_base_obs_dist_hard_thres.publish(poly_msg)

        # (x,y,theta) = self.get_2d_pose_from_tf(self.TFs[self.index])
        # # Rotate & Translate the polygon to its pose in world frame
        # r = shapely.affinity.rotate(s, theta, origin=(0.,0.), use_radians=True)
        # r = shapely.affinity.translate(r, xoff = x, yoff = y)
        # rospy.logwarn("Collision Avoidance 2D: obs_dist_hard_thres_polygon corners xy:\n" + '\n'.join(map(str, list(r.exterior.coords))) )
        self.obs_dist_hard_thres_polygon = r

    def cmd_vel_callback(self,msg):
        self.time_last_cmd_vel = rospy.Time.now().to_sec()
        self.Vx = msg.linear.x
        self.Vy = msg.linear.y
        self.Vz = msg.linear.z
        self.Wx = msg.angular.x
        self.Wy = msg.angular.y
        self.Wz = msg.angular.z

    def laser_obstacles_callback(self, msg):
        self.time_last_laser_obstacles = rospy.Time.now().to_sec()
        self.laser_obstacles_msg = msg

    def calculate_obstacles_dynamic(self):
        if (self.enable_collision_avoidance_dynamic) and (self.laser_obstacles_msg is not None):
            collision_polygons_dynamic = [] # list of detected collisions' shapely Point objects
            collision_polygons_dynamic_hard = [] # list of detected hard collisions' shapely Point objects

            lines = [] # list of rectangular shapely polygons
            for line in self.laser_obstacles_msg.segments:
                line_str = shapely.geometry.LineString([ (line.first_point.x,line.first_point.y), (line.last_point.x,line.last_point.y) ])
                # Enlarge the line segments to rectangles with shapely
                line_rect = line_str.buffer(self.line_obstacle_buffer_distance, cap_style=2,join_style=2)
                lines.append(line_rect)

            circles = [] # list of circular shapely polygons
            for circle in self.laser_obstacles_msg.circles:
                center = circle.center
                radius = circle.radius # Use the circles with safety margin as radius
                # Use shapely's buffer function to crate circles
                circle_poly = shapely.geometry.Point(center.x, center.y).buffer(radius)
                circles.append(circle_poly)

            laser_obstacles = lines + circles

            for laser_obstacle in laser_obstacles:
                # Check whether the line is in the obstacle threshold zone
                if laser_obstacle.overlaps(self.obs_dist_thres_polygon):
                    # Find the polygon at the interior of the self.obs_dist_thres_polygon
                    collision_polygon = self.obs_dist_thres_polygon.intersection(laser_obstacle)

                    # if this collision polygon is also at the interior of the obs_dist_hard_thres_polygon, we need s special treat!
                    if laser_obstacle.overlaps(self.obs_dist_hard_thres_polygon):
                        collision_polygon = self.obs_dist_hard_thres_polygon.intersection(laser_obstacle)

                        if collision_polygon.geom_type == 'MultiPolygon':
                            for obj in collision_polygon.geoms:
                                if obj.geom_type == 'Polygon':
                                    if not self.is_convex_polygon(list(obj.exterior.coords)[:-1]):
                                        # rospy.logerr("Hard threshold non-convex obstacle for hitting laser scanner obstacle is triggered!!")
                                        for obj_partial in triangulate(obj):
                                            if obj.contains(obj_partial):
                                                collision_polygons_dynamic_hard.append(obj_partial)
                                    else:
                                        collision_polygons_dynamic_hard.append(obj)

                        elif collision_polygon.geom_type == 'Polygon':
                            if not self.is_convex_polygon(list(collision_polygon.exterior.coords)[:-1]):
                                # rospy.logerr("Hard threshold non-convex obstacle for hitting laser scanner obstacle is triggered!!")
                                for obj_partial in triangulate(collision_polygon):
                                    if collision_polygon.contains(obj_partial):
                                        collision_polygons_dynamic_hard.append(obj_partial)
                            else:
                                collision_polygons_dynamic_hard.append(collision_polygon)

                    else:
                        if collision_polygon.geom_type == 'MultiPolygon':
                            for obj in collision_polygon.geoms:
                                if obj.geom_type == 'Polygon':
                                    if not self.is_convex_polygon(list(obj.exterior.coords)[:-1]):
                                        # rospy.logerr("Soft threshold non-convex obstacle for hitting other robots is triggered!!")
                                        for obj_partial in triangulate(obj):
                                            if obj.contains(obj_partial):
                                                collision_polygons_dynamic.append(obj_partial)
                                    else:
                                        collision_polygons_dynamic.append(obj)

                        elif collision_polygon.geom_type == 'Polygon':
                            if not self.is_convex_polygon(list(collision_polygon.exterior.coords)[:-1]):
                                # rospy.logerr("Soft threshold non-convex obstacle for hitting other robots is triggered!!")
                                for obj_partial in triangulate(collision_polygon):
                                    if collision_polygon.contains(obj_partial):
                                        collision_polygons_dynamic.append(obj_partial)
                            else:
                                collision_polygons_dynamic.append(collision_polygon)

            return collision_polygons_dynamic, collision_polygons_dynamic_hard 
        else:
            return [], []


    def calculate_obstacles(self):
        # self.workspace_polygon 
        # self.mobile_base_polygons[] 
        # self.obs_dist_thres_polygon 
        # self.obs_dist_hard_thres_polygon

        collision_polygons = [] # list of detected collisions' shapely Polygon objects
        collision_polygons_hard = [] # list of detected hard collisions' shapely Polygon objects

        # Check if the robot is colliding with the workspace 
        # (ie. is the robot obs thresh within the workspace_polygon)
          
        if not self.workspace_polygon.contains(self.obs_dist_thres_polygon):
            # contains: no points of self.obs_dist_thres_polygon lie in the exterior of self.workspace_polygon 
            # and at least one point of the interior of self.obs_dist_thres_polygon lies in the interior of self.workspace_polygon.
            
            # ie. Find the polygon at the exterior of the workspace_polygon
            collision_polygon = self.obs_dist_thres_polygon.difference(self.workspace_polygon)

            # if this collision polygon is also at the interior of the obs_dist_hard_thres_polygon, we need a special treat!
            if not self.workspace_polygon.contains(self.obs_dist_hard_thres_polygon):
                # ie. Find the polygon at the exterior of the workspace_polygon
                collision_polygon = self.obs_dist_hard_thres_polygon.difference(self.workspace_polygon)

                if collision_polygon.geom_type == 'MultiPolygon':
                    for obj in collision_polygon.geoms:
                        if obj.geom_type == 'Polygon':
                            if not self.is_convex_polygon(list(obj.exterior.coords)[:-1]):
                                # rospy.logerr("Hard threshold non-convex obstacle for workspace is triggered!!")
                                for obj_partial in triangulate(obj):
                                    if obj.contains(obj_partial):
                                        collision_polygons_hard.append(obj_partial)
                            else:
                                collision_polygons_hard.append(obj)
                            

                elif collision_polygon.geom_type == 'Polygon':
                    if not self.is_convex_polygon(list(collision_polygon.exterior.coords)[:-1]):
                        # rospy.logerr("Hard threshold non-convex obstacle for workspace is triggered!!")
                        for obj_partial in triangulate(collision_polygon):
                            if collision_polygon.contains(obj_partial):
                                collision_polygons_hard.append(obj_partial)
                    else:
                        collision_polygons_hard.append(collision_polygon)
                    
            
                # rospy.logwarn("Hard threshold for workspace is triggered")
                # rospy.logwarn(str(collision_polygon.geom_type))    

            else:
                if collision_polygon.geom_type == 'MultiPolygon':
                    for obj in collision_polygon.geoms:
                        if obj.geom_type == 'Polygon':
                            if not self.is_convex_polygon(list(obj.exterior.coords)[:-1]):
                                # rospy.logerr("Soft threshold non-convex obstacle!!")
                                for obj_partial in triangulate(obj):
                                    if obj.contains(obj_partial):
                                        collision_polygons.append(obj_partial)
                            else:
                                collision_polygons.append(obj)

                elif collision_polygon.geom_type == 'Polygon':
                    if not self.is_convex_polygon(list(collision_polygon.exterior.coords)[:-1]):
                        # rospy.logerr("Soft threshold non-convex obstacle for workspace is triggered!!")
                        for obj_partial in triangulate(collision_polygon):
                            if collision_polygon.contains(obj_partial):
                                collision_polygons.append(obj_partial)
                    else:
                        collision_polygons.append(collision_polygon)

                # rospy.logwarn("Soft threshold for workspace is triggered")
                # rospy.logwarn(str(collision_polygon.geom_type))    

        # Check if the robot is colliding with any of the other robots
        for i in range(self.num_of_robots):      
            if i != self.index:
                # rospy.logwarn(str(self.mobile_base_polygons[i].overlaps(self.obs_dist_thres_polygon)))      
                if self.mobile_base_polygons[i].overlaps(self.obs_dist_thres_polygon):
                    # within: if self.mobile_base_polygons[i]'s boundary and interior intersect only with the interior of the self.obs_dist_thres_polygon (not its boundary or exterior).
                    # (intersects: they have any boundary or interior point in common)
                    # overlaps: the geometries have more than one but not all points in common, have the same dimension, and the intersection of the interiors of the geometries has the same dimension as the geometries themselves

                    # Find the polygon at the interior of the self.obs_dist_thres_polygon
                    collision_polygon = self.obs_dist_thres_polygon.intersection(self.mobile_base_polygons[i])

                    # if this collision polygon is also at the interior of the obs_dist_hard_thres_polygon, we need s special treat!
                    if self.mobile_base_polygons[i].overlaps(self.obs_dist_hard_thres_polygon):
                        # ie. Find the polygon at the exterior of the workspace_polygon
                        collision_polygon = self.obs_dist_hard_thres_polygon.intersection(self.mobile_base_polygons[i])

                        if collision_polygon.geom_type == 'MultiPolygon':
                            for obj in collision_polygon.geoms:
                                if obj.geom_type == 'Polygon':
                                    if not self.is_convex_polygon(list(obj.exterior.coords)[:-1]):
                                        # rospy.logerr("Hard threshold non-convex obstacle for hitting other robots is triggered!!")
                                        for obj_partial in triangulate(obj):
                                            if obj.contains(obj_partial):
                                                collision_polygons_hard.append(obj_partial)
                                    else:
                                        collision_polygons_hard.append(obj)

                        elif collision_polygon.geom_type == 'Polygon':
                            if not self.is_convex_polygon(list(collision_polygon.exterior.coords)[:-1]):
                                # rospy.logerr("Hard threshold non-convex obstacle for hitting other robots is triggered!!")
                                for obj_partial in triangulate(collision_polygon):
                                    if collision_polygon.contains(obj_partial):
                                        collision_polygons_hard.append(obj_partial)
                            else:
                                collision_polygons_hard.append(collision_polygon)

                        
                        # rospy.logwarn("Hard threshold for hitting other robots is triggered")
                        # rospy.logwarn(str(collision_polygon.geom_type))   

                    else:
                        if collision_polygon.geom_type == 'MultiPolygon':
                            for obj in collision_polygon.geoms:
                                if obj.geom_type == 'Polygon':
                                    if not self.is_convex_polygon(list(obj.exterior.coords)[:-1]):
                                        # rospy.logerr("Soft threshold non-convex obstacle for hitting other robots is triggered!!")
                                        for obj_partial in triangulate(obj):
                                            if obj.contains(obj_partial):
                                                collision_polygons.append(obj_partial)
                                    else:
                                        collision_polygons.append(obj)

                        elif collision_polygon.geom_type == 'Polygon':
                            if not self.is_convex_polygon(list(collision_polygon.exterior.coords)[:-1]):
                                # rospy.logerr("Soft threshold non-convex obstacle for hitting other robots is triggered!!")
                                for obj_partial in triangulate(collision_polygon):
                                    if collision_polygon.contains(obj_partial):
                                        collision_polygons.append(obj_partial)
                            else:
                                collision_polygons.append(collision_polygon)


                        # rospy.logwarn("Soft threshold for hitting other robots is triggered")
                        # rospy.logwarn(str(collision_polygon.geom_type))   

        return collision_polygons, collision_polygons_hard

    def avoid_obstacles(self, collision_polygons, collision_polygons_hard, collision_polygons_dynamic, collision_polygons_dynamic_hard):
        # Note that collision_polygons or/and collision_polygons_hard are lists
        # and include shapely Polygon objects
        V = np.array([self.Vx,self.Vy])
        W = np.array([self.Wz])

        forces = []
        torques = []

        # Closest points to the obstacles on the robot mobile base frame polygon
        for polygon in collision_polygons:
            nearest_pts = nearest_points(self.mobile_base_polygons[self.index], polygon)
            pt_on_self = np.squeeze(np.asarray(nearest_pts[0].coords)) # closest point on the robot # array([x, y])
            pt_on_obj =  np.squeeze(np.asarray(nearest_pts[1].coords)) # closest point on the obstacle object # array([x, y])
            dist = self.mobile_base_polygons[self.index].distance(polygon) # distance between the obstacle object and the robot # float
            if dist > 0.0:
                unit_vect = (pt_on_self - pt_on_obj) / dist # unit vector from obstacle object to the robot # array([x, y]) 

                # factor is btw [0,1]; 
                # 0: distance is too far away (away from the specified obs_dist_thres), 
                # 1: distance is too close (closer than the specified obs_dist_hard_thres)
                factor = 1.0 - (  (dist - self.obs_dist_hard_thres) / (self.obs_dist_thres - self.obs_dist_hard_thres) ) # linearly changing factor

                # Calculate the linear repulsive "force" caused by the obstacle acting on the robot multiplied by its factor
                force = factor * unit_vect
                forces.append(force)

                # Calculate repulsive "torque" caused by the obstacle acting on the robot multiplied by its factor
                # Similar to the normalization of the force norm btw. 0 to 1, with the hard and soft threshold values, 
                # we also need to scale the maximum possible torque btw 0 to 1.
                # We can achieve that finding the possible max and min distances that a torque can be applied to the origin of the robot.
                # Shapely has distance(for min) and hausdorff_distance(for max) values.
                pt_center = shapely.geometry.Point(0,0) # assumed to be the origin of the robot and inside polygon of the robot
                # min_dist = pt_center.distance(self.mobile_base_polygons[self.index])
                # max_dist = pt_center.hausdorff_distance(self.mobile_base_polygons[self.index])
                app_dist = pt_center.distance(nearest_pts[0]) # point of application of the force/torque distance to the pt_center
                unit_vect_torque = pt_on_self / app_dist

                # factor_torque = (app_dist - min_dist) / (max_dist - min_dist)
                
                # r = factor_torque * unit_vect_torque
                r = unit_vect_torque
                torque = float(np.cross(r,force)) # on 2D, numpy cross returns a scalar array, convert it to float
                torques.append(torque)
            else:
                # rospy.logwarn("Soft threshold Obstacle collided with the robot!!")
                rospy.logwarn_throttle(20.0, "Soft threshold Obstacle collided with the robot!! (throttled to 20.0s)")

        forces_dynamic = []
        torques_dynamic = []

        for polygon in collision_polygons_dynamic:
            nearest_pts = nearest_points(self.mobile_base_polygons[self.index], polygon)
            pt_on_self = np.squeeze(np.asarray(nearest_pts[0].coords)) # closest point on the robot # array([x, y])
            pt_on_obj =  np.squeeze(np.asarray(nearest_pts[1].coords)) # closest point on the obstacle object # array([x, y])
            dist = self.mobile_base_polygons[self.index].distance(polygon) # distance between the obstacle object and the robot # float
            if dist > 0.0:
                unit_vect = (pt_on_self - pt_on_obj) / dist # unit vector from obstacle object to the robot # array([x, y]) 
                

                # factor is btw [0,1]; 
                # 0: distance is too far away (away from the specified obs_dist_thres), 
                # 1: distance is too close (closer than the specified obs_dist_hard_thres)
                factor = 1.0 - (  (dist - self.obs_dist_hard_thres) / (self.obs_dist_thres - self.obs_dist_hard_thres) ) # linearly changing factor

                # Calculate the linear repulsive "force" caused by the obstacle acting on the robot multiplied by its factor
                force = factor * unit_vect
                forces_dynamic.append(force)

                # Calculate repulsive "torque" caused by the obstacle acting on the robot multiplied by its factor
                # Similar to the normalization of the force norm btw. 0 to 1, with the hard and soft threshold values, 
                # we also need to scale the maximum possible torque btw 0 to 1.
                # We can achieve that finding the possible max and min distances that a torque can be applied to the origin of the robot.
                # Shapely has distance(for min) and hausdorff_distance(for max) values.
                pt_center = shapely.geometry.Point(0,0) # assumed to be the origin of the robot and inside polygon of the robot
                # min_dist = pt_center.distance(self.mobile_base_polygons[self.index])
                # max_dist = pt_center.hausdorff_distance(self.mobile_base_polygons[self.index])
                app_dist = pt_center.distance(nearest_pts[0]) # point of application of the force/torque distance to the pt_center
                unit_vect_torque = pt_on_self / app_dist

                # factor_torque = (app_dist - min_dist) / (max_dist - min_dist)
                
                # r = factor_torque * unit_vect_torque
                r = unit_vect_torque
                torque = float(np.cross(r,force)) # on 2D, numpy cross returns a scalar array, convert it to float
                torques_dynamic.append(torque)
            else:
                # rospy.logwarn("Soft threshold Obstacle from Laser Scanner collided with the robot!!")
                rospy.logwarn_throttle(20.0, "Soft threshold Obstacle from Laser Scanner collided with the robot!! (throttled to 20.0s)")

        # forces_dynamic = np.array(forces_dynamic)
        # torques_dynamic = np.array(torques_dynamic)

        # force_dynamic_sum = np.sum(forces_dynamic,axis=0) # summation of all factored force vectors with norms btw 0-1. # array([x, y])
        # torque_dynamic_sum = np.sum(torques_dynamic,axis=0) # summation of all factored torques with norms btw 0-1. (scalar bcs. all torques are in 2D and creates a vector around z axis)

        # if len(forces_dynamic) > 0:
        #     forces_dynamic_avr = force_dynamic_sum / len(forces_dynamic)  # array([x, y])
        # else:
        #     forces_dynamic_avr = force_dynamic_sum  # array([x, y])

        # if len(torques_dynamic) > 0:
        #     torques_dynamic_avr = torque_dynamic_sum / len(torques_dynamic) # scalar float
        # else:
        #     torques_dynamic_avr = torque_dynamic_sum # scalar float
        
        # Add the average of dynamic forces to other forces with some weighting
        # weight_forces_dynamic = 1.0
        # weight_torques_dynamic = 1.0
        # forces.append(weight_forces_dynamic   * forces_dynamic_avr)
        # torques.append(weight_torques_dynamic * torques_dynamic_avr)

        forces = forces + forces_dynamic # join two lists
        torques = torques + torques_dynamic # join two lists

        # Now continue the total force calculation
        forces = np.array(forces)
        torques = np.array(torques)

        force_sum = np.sum(forces,axis=0) # summation of all factored force vectors with norms btw 0-1. # array([x, y])
        torque_sum = np.sum(torques,axis=0) # summation of all factored torques with norms btw 0-1. (scalar bcs. all torques are in 2D and creates a vector around z axis)

        if len(forces) > 0:
            force_avr = force_sum / len(forces)  # array([x, y])
        else:
            force_avr = force_sum  # array([x, y])

        if len(torques) > 0:
            torque_avr = torque_sum / len(torques) # scalar float
        else:
            torque_avr = torque_sum # scalar float

        force_avr_norm = np.linalg.norm(force_avr)
        torque_avr_norm = np.linalg.norm(torque_avr)
        # rospy.logwarn('force_avr_norm: '+ str(force_avr_norm) + ", torque_avr_norm:" + str(torque_avr_norm) )
        n = 0.1

        factor_v = min(max(force_avr_norm, 0.0), 1.0) # btw 0 to 1, 1 eliminates all velocity towards a directions, 0 does not eliminate anything
        if (force_avr_norm > 0) and (np.dot(V,force_avr) < 0.0):
            V = V - (factor_v)**n * ((np.dot(V,force_avr) * force_avr) / force_avr_norm**2)
      
        factor_w = min(max(torque_avr_norm, 0.0), 1.0) # btw 0 to 1, 1 eliminates all velocity towards a directions, 0 does not eliminate anything
        if (torque_avr_norm > 0) and (np.dot(W,torque_avr) < 0.0):
            W = W - (factor_w)**n * ((np.dot(W,torque_avr) * torque_avr) / torque_avr_norm**2)

        ##########################################################################
        # After the soft threshold obstacles are treated, if there is a hard threshold obstacle, we need to create a repulsive velocity out of it.
        forces = []
        torques = []

        for polygon in collision_polygons_hard: 
            nearest_pts = nearest_points(self.mobile_base_polygons[self.index], polygon)
            pt_on_self = np.squeeze(np.asarray(nearest_pts[0].coords)) # closest point on the robot # array([x, y])
            pt_on_obj =  np.squeeze(np.asarray(nearest_pts[1].coords)) # closest point on the obstacle object # array([x, y])
            dist = self.mobile_base_polygons[self.index].distance(polygon) # distance between the obstacle object and the robot # float
            if dist > 0.0:
                unit_vect = (pt_on_self - pt_on_obj) / dist # unit vector from obstacle object to the robot # array([x, y]) 

                # factor is btw [0,1]; 
                # 0: distance is too far away (away from the specified obs_dist_thres), 
                # 1: distance is too close (closer than the specified obs_dist_hard_thres)
                factor = 1.0 - (  dist / self.obs_dist_hard_thres )**2 # quadratically changing factor

                # Calculate the linear repulsive "force" caused by the obstacle acting on the robot multiplied by its factor
                force = factor * unit_vect 
                forces.append(force)       

                # Calculate repulsive "torque" caused by the obstacle acting on the robot multiplied by its factor
                # Similar to the normalization of the force norm btw. 0 to 1, with the hard and soft threshold values, 
                # we also need to scale the maximum possible torque btw 0 to 1.
                # We can achieve that finding the possible max and min distances that a torque can be applied to the origin of the robot.
                # Shapely has distance(for min) and hausdorff_distance(for max) values.
                pt_center = shapely.geometry.Point(0,0) # assumed to be the origin of the robot and inside polygon of the robot
                # min_dist = pt_center.distance(self.mobile_base_polygons[self.index])
                # max_dist = pt_center.hausdorff_distance(self.mobile_base_polygons[self.index])
                app_dist = pt_center.distance(nearest_pts[0]) # point of application of the force/torque distance to the pt_center
                unit_vect_torque = pt_on_self / app_dist

                # factor_torque = (app_dist - min_dist) / (max_dist - min_dist)
                
                # r = factor_torque * unit_vect_torque
                r = unit_vect_torque
                torque = float(np.cross(r,force)) # on 2D, numpy cross returns a scalar array, convert it to float
                torques.append(torque)
            else:
                # rospy.logwarn("Obstacle collided with the robot!!")
                rospy.logwarn_throttle(20.0, "Obstacle collided with the robot!! (throttled to 20.0s)")

        forces_dynamic = []
        torques_dynamic = []

        for polygon in collision_polygons_dynamic_hard: 
            nearest_pts = nearest_points(self.mobile_base_polygons[self.index], polygon)
            pt_on_self = np.squeeze(np.asarray(nearest_pts[0].coords)) # closest point on the robot # array([x, y])
            pt_on_obj =  np.squeeze(np.asarray(nearest_pts[1].coords)) # closest point on the obstacle object # array([x, y])
            dist = self.mobile_base_polygons[self.index].distance(polygon) # distance between the obstacle object and the robot # float
            if dist > 0.0:
                unit_vect = (pt_on_self - pt_on_obj) / dist # unit vector from obstacle object to the robot # array([x, y]) 

                # factor is btw [0,1]; 
                # 0: distance is too far away (away from the specified obs_dist_thres), 
                # 1: distance is too close (closer than the specified obs_dist_hard_thres)
                factor = 1.0 - (  dist / self.obs_dist_hard_thres )**2 # quadratically changing factor

                # Calculate the linear repulsive "force" caused by the obstacle acting on the robot multiplied by its factor
                force = factor * unit_vect 
                forces_dynamic.append(force)       

                # Calculate repulsive "torque" caused by the obstacle acting on the robot multiplied by its factor
                # Similar to the normalization of the force norm btw. 0 to 1, with the hard and soft threshold values, 
                # we also need to scale the maximum possible torque btw 0 to 1.
                # We can achieve that finding the possible max and min distances that a torque can be applied to the origin of the robot.
                # Shapely has distance(for min) and hausdorff_distance(for max) values.
                pt_center = shapely.geometry.Point(0,0) # assumed to be the origin of the robot and inside polygon of the robot
                # min_dist = pt_center.distance(self.mobile_base_polygons[self.index])
                # max_dist = pt_center.hausdorff_distance(self.mobile_base_polygons[self.index])
                app_dist = pt_center.distance(nearest_pts[0]) # point of application of the force/torque distance to the pt_center
                unit_vect_torque = pt_on_self / app_dist

                # factor_torque = (app_dist - min_dist) / (max_dist - min_dist)
                
                # r = factor_torque * unit_vect_torque
                r = unit_vect_torque
                torque = float(np.cross(r,force)) # on 2D, numpy cross returns a scalar array, convert it to float
                torques_dynamic.append(torque)
            else:
                # rospy.logwarn("Obstacle from Laser Scanner collided with the robot!!")
                rospy.logwarn_throttle(20.0, "Obstacle from Laser Scanner collided with the robot!! (throttled to 20.0s)")

        # forces_dynamic = np.array(forces_dynamic)
        # torques_dynamic = np.array(torques_dynamic)

        # force_dynamic_sum = np.sum(forces_dynamic,axis=0) # summation of all factored force vectors with norms btw 0-1. # array([x, y])
        # torque_dynamic_sum = np.sum(torques_dynamic,axis=0) # summation of all factored torques with norms btw 0-1. (scalar bcs. all torques are in 2D and creates a vector around z axis)

        # if len(forces_dynamic) > 0:
        #     forces_dynamic_avr = force_dynamic_sum / len(forces_dynamic)  # array([x, y])
        # else:
        #     forces_dynamic_avr = force_dynamic_sum  # array([x, y])

        # if len(torques_dynamic) > 0:
        #     torques_dynamic_avr = torque_dynamic_sum / len(torques_dynamic) # scalar float
        # else:
        #     torques_dynamic_avr = torque_dynamic_sum # scalar float
        
        # # Add the average of dynamic forces to other forces with some weighting
        # weight_forces_dynamic = 1.0
        # weight_torques_dynamic = 1.0
        # forces.append(weight_forces_dynamic   * forces_dynamic_avr)
        # torques.append(weight_torques_dynamic * torques_dynamic_avr)

        forces = forces + forces_dynamic # join two lists
        torques = torques + torques_dynamic # join two lists

        # Now continue the total force calculation
        forces = np.array(forces)
        torques = np.array(torques)

        force_sum = np.sum(forces,axis=0) # summation of all factored force vectors with norms btw 0-1. # array([x, y])
        torque_sum = np.sum(torques,axis=0) # summation of all factored torques with norms btw 0-1. (scalar bcs. all torques are in 2D and creates a vector around z axis)

        if len(forces) > 0:
            force_avr = force_sum / len(forces)  # array([x, y])
        else:
            force_avr = force_sum  # array([x, y])

        if len(torques) > 0:
            torque_avr = torque_sum / len(torques) # scalar float
        else:
            torque_avr = torque_sum # scalar float

        force_avr_norm = np.linalg.norm(force_avr)
        torque_avr_norm = np.linalg.norm(torque_avr)
        # rospy.logwarn('force_avr_norm: '+ str(force_avr_norm) + ", torque_avr_norm:" + str(torque_avr_norm) )

        factor_v = min(max(force_avr_norm, 0.0), 1.0) # btw 0 to 1, 1 eliminates all velocity towards a directions, 0 does not eliminate anything
        if (force_avr_norm > 0) and (np.dot(V,force_avr) < 0.0):
            V = V - (1.0+ factor_v**2) * ((np.dot(V,force_avr) * force_avr) / force_avr_norm**2)
      
        factor_w = min(max(torque_avr_norm, 0.0), 1.0) # btw 0 to 1, 1 eliminates all velocity towards a directions, 0 does not eliminate anything
        if (torque_avr_norm > 0) and (np.dot(W,torque_avr) < 0.0):
            W = W - (1.0+ factor_w**2) * ((np.dot(W,torque_avr) * torque_avr) / torque_avr_norm**2)

        # self.Vx = V[0]
        # self.Vy = V[1]
        # self.Wz = W[0]

        # filtering the velocities
        filter_coeff = 1.0 # 1: NO filtering
        self.Vx_modified = (1-filter_coeff) * self.Vx_modified + filter_coeff * V[0]
        self.Vy_modified = (1-filter_coeff) * self.Vy_modified + filter_coeff * V[1]
        self.Wz_modified = (1-filter_coeff) * self.Wz_modified + filter_coeff * W[0]
    
    def publishVelCmd(self):
        vel_msg = geometry_msgs.msg.Twist()
        # vel_msg.linear.x  = self.Vx
        # vel_msg.linear.y  = self.Vy
        # vel_msg.linear.z  = self.Vz
        # vel_msg.angular.x = self.Wx
        # vel_msg.angular.y = self.Wy
        # vel_msg.angular.z = self.Wz
        vel_msg.linear.x  = self.Vx_modified
        vel_msg.linear.y  = self.Vy_modified
        vel_msg.linear.z  = self.Vz_modified
        vel_msg.angular.x = self.Wx_modified
        vel_msg.angular.y = self.Wy_modified
        vel_msg.angular.z = self.Wz_modified
        self.pub_cmd_vel.publish(vel_msg)

    def srv_toggle_collision_avoidance_cb(self,req):
        assert isinstance(req, SetBoolRequest)

        if req.data:
            self.enable_collision_avoidance = True
        else:
            self.enable_collision_avoidance = False

        return SetBoolResponse(True, "The collision_avoidance is now set to: {}".format(self.enable_collision_avoidance))



    # UTIL functions below:
    def get_2d_pose_from_tf(self,tf):
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        z = tf.transform.translation.z

        qw = tf.transform.rotation.w # Scalar part of quaternion
        qx = tf.transform.rotation.x
        qy = tf.transform.rotation.y
        qz = tf.transform.rotation.z
        # Euler XYZ
        # orientation_error = tf_conversions.transformations.euler_from_quaternion(q_orientation_error)
        roll_x, pitch_y, yaw_z = self.euler_from_quaternion(qx, qy, qz, qw) # in radians
        return (x,y,yaw_z)

    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

    

    def is_convex_polygon(self,polygon):
        # retrieved from
        # https://stackoverflow.com/questions/471962/how-do-i-efficiently-determine-if-a-polygon-is-convex-non-convex-or-complex/45372025#45372025

        TWO_PI = 2 * math.pi
        """Return True if the polynomial defined by the sequence of 2D
        points is 'strictly convex': points are valid, side lengths non-
        zero, interior angles are strictly between zero and a straight
        angle, and the polygon does not intersect itself.

        NOTES:  1.  Algorithm: the signed changes of the direction angles
                    from one side to the next side must be all positive or
                    all negative, and their sum must equal plus-or-minus
                    one full turn (2 pi radians). Also check for too few,
                    invalid, or repeated points.
                2.  No check is explicitly done for zero internal angles
                    (180 degree direction-change angle) as this is covered
                    in other ways, including the `n < 3` check.
        """
        try:  # needed for any bad points or direction changes
            # Check for too few points
            if len(polygon) < 3:
                # rospy.logwarn("Here1")
                return False
            # Get starting information
            old_x, old_y = polygon[-2]
            new_x, new_y = polygon[-1]
            new_direction = math.atan2(new_y - old_y, new_x - old_x)
            angle_sum = 0.0
            # Check each point (the side ending there, its angle) and accum. angles
            for ndx, newpoint in enumerate(polygon):
                # Update point coordinates and side directions, check side length
                old_x, old_y, old_direction = new_x, new_y, new_direction
                new_x, new_y = newpoint
                new_direction = math.atan2(new_y - old_y, new_x - old_x)
                if old_x == new_x and old_y == new_y:
                    # rospy.logwarn("Here2")
                    return False  # repeated consecutive points

                # Calculate & check the normalized direction-change angle
                angle = new_direction - old_direction
                if angle <= -math.pi:
                    angle += TWO_PI  # make it in half-open interval (-Pi, Pi]
                elif angle > math.pi:
                    angle -= TWO_PI
                if ndx == 0:  # if first time through loop, initialize orientation
                    if angle == 0.0:
                        # rospy.logwarn("Here3")
                        return False
                    orientation = 1.0 if angle > 0.0 else -1.0
                else:  # if other time through loop, check orientation is stable
                    if orientation * angle <= 0.0:  # not both pos. or both neg.
                        # rospy.logwarn("Here4")
                        return False
                # Accumulate the direction-change angle
                angle_sum += angle
            # Check that the total number of full turns is plus-or-minus 1
            return abs(round(angle_sum / TWO_PI)) == 1
        except: #(ArithmeticError, TypeError, ValueError):
            # rospy.logwarn("Here5")
            return False  # any exception means not a proper convex polygon


    


        

if __name__ == "__main__":
    collisionAvoidance2D = CollisionAvoidance2D()
    rospy.spin()
