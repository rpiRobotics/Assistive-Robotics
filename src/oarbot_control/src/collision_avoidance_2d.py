#!/usr/bin/env python

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

import shapely
import shapely.geometry
import numpy as np

class CollisionAvoidance2D():
    def __init__(self):

        rospy.init_node('collision_avoidance_2d', anonymous=True)

        # Published topic names 
        self.out_cmd_vel_topic_name = rospy.get_param("~out_cmd_vel_topic_name")
        # Publishers
        self.pub_cmd_vel = rospy.Publisher(self.out_cmd_vel_topic_name, geometry_msgs.msg.Twist, queue_size=2)

        # Subscribed topic names
        self.in_cmd_vel_topic_name = rospy.get_param("~in_cmd_vel_topic_name")
        # Subscribers
        # self.sub_cmd_vel = rospy.Subscriber(self.in_cmd_vel_topic_name, geometry_msgs.msg.Twist, self.TODO, queue_size=None)

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
        
        # Publish rate debuggers/visualizers
        self.viz_out_rate = rospy.get_param("~viz_out_rate", 10.0) 

        # Start publishing
        rospy.Timer(rospy.Duration(1.00/self.viz_out_rate), self.publish_visualizers)

    def publish_visualizers(self, event=None):
        # Find the transforms between the robots and the world

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
        self.workspace_polygon = shapely.geometry.Polygon(r)

    def create_mobile_base_frame_polygons(self):
        for i in range(self.num_of_robots):
            poly_msg = geometry_msgs.msg.PolygonStamped()

            poly_msg.header.stamp = rospy.Time.now()
            poly_msg.header.frame_id = self.all_tf_mobile_base_frame_ids[i]

            poly_msg.polygon.points = [geometry_msgs.msg.Point32(x=corner[0], y=corner[1]) for corner in self.all_mobile_base_frame_coords[i]] 

            self.pubs_viz_mobile_base_polygon[i].publish(poly_msg)

            r = shapely.geometry.LinearRing(self.all_mobile_base_frame_coords[i])
            self.mobile_base_polygons[i] = shapely.geometry.Polygon(r)
            

    def create_obs_dist_thres(self):
        poly_msg = geometry_msgs.msg.PolygonStamped()

        poly_msg.header.stamp = rospy.Time.now()
        poly_msg.header.frame_id = self.tf_mobile_base_frame_id

        r = shapely.geometry.LinearRing(self.all_mobile_base_frame_coords[self.index])
        s = shapely.geometry.Polygon(r)
        self.obs_dist_thres_polygon = shapely.geometry.Polygon(s.buffer(self.obs_dist_thres).exterior, [r])
        Xs = list(self.obs_dist_thres_polygon.exterior.coords.xy[0])
        Ys = list(self.obs_dist_thres_polygon.exterior.coords.xy[1])
        
        poly_msg.polygon.points = [geometry_msgs.msg.Point32(x=Xs[i], y=Ys[i]) for i in range(len(Xs)-1)] # do not include the last point since it is the same with the forst point

        self.pub_viz_mobile_base_obs_dist_thres.publish(poly_msg)

    def create_obs_dist_hard_thres(self):
        poly_msg = geometry_msgs.msg.PolygonStamped()

        poly_msg.header.stamp = rospy.Time.now()
        poly_msg.header.frame_id = self.tf_mobile_base_frame_id

        r = shapely.geometry.LinearRing(self.all_mobile_base_frame_coords[self.index])
        s = shapely.geometry.Polygon(r)
        self.obs_dist_hard_thres_polygon = shapely.geometry.Polygon(s.buffer(self.obs_dist_hard_thres).exterior, [r])
        Xs = list(self.obs_dist_hard_thres_polygon.exterior.coords.xy[0])
        Ys = list(self.obs_dist_hard_thres_polygon.exterior.coords.xy[1])
        
        poly_msg.polygon.points = [geometry_msgs.msg.Point32(x=Xs[i], y=Ys[i]) for i in range(len(Xs)-1)] # do not include the last point since it is the same with the forst point

        self.pub_viz_mobile_base_obs_dist_hard_thres.publish(poly_msg)

    

if __name__ == "__main__":
    collisionAvoidance2D = CollisionAvoidance2D()
    rospy.spin()
