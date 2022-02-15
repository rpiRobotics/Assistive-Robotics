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
import math

class CollisionAvoidance2D():
    def __init__(self):

        rospy.init_node('collision_avoidance_2d', anonymous=True)

        # Published topic names 
        self.out_cmd_vel_topic_name = rospy.get_param("~out_cmd_vel_topic_name")
        # Publishers
        self.pub_cmd_vel = rospy.Publisher(self.out_cmd_vel_topic_name, geometry_msgs.msg.Twist, queue_size=2)

        # To store the velocities
        self.Vx = 0.0
        self.Vy = 0.0
        self.Vz = 0.0
        self.Wx = 0.0
        self.Wy = 0.0
        self.Wz = 0.0

        # Subscribed topic names
        self.in_cmd_vel_topic_name = rospy.get_param("~in_cmd_vel_topic_name")
        # Subscribers
        self.sub_cmd_vel = rospy.Subscriber(self.in_cmd_vel_topic_name, geometry_msgs.msg.Twist, self.cmd_vel_callback, queue_size=1)

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
        
        # Publish rate debuggers/visualizers
        self.viz_out_rate = rospy.get_param("~viz_out_rate", 100.0) 

        # Start publishing
        rospy.Timer(rospy.Duration(1.00/self.viz_out_rate), self.run)

    def run(self, event=None):
        self.TFs_are_ready = self.get_TFs()

        if self.TFs_are_ready:
            # Calculate shapely polygons and publish the visualizers
            self.publish_visualizers()

            # Calculate the obstacles with shapely
            collision_polygons, collision_polygons_hard = self.calculate_obstacles()
            
            # Publish the safer cmd_vel
            # TODO
            
        else:
            # Publish zero velocities ?
            pass

    

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
                rospy.logwarn('Collision Avoidance 2D: Could not find the transformation from %s to (%s or %s)' 
                                % (self.tf_mobile_base_frame_id, self.tf_world_frame_id, self.all_tf_mobile_base_frame_ids[i])) 

                # reset the saved TFs to None
                rospy.logwarn('Collision Avoidance 2D: Resetting the saved TF list')
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
        self.Vx = msg.linear.x
        self.Vy = msg.linear.y
        self.Vz = msg.linear.z
        self.Wx = msg.angular.x
        self.Wy = msg.angular.y
        self.Wz = msg.angular.z

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

    def calculate_obstacles(self):
        # self.workspace_polygon 
        # self.mobile_base_polygons[] 
        # self.obs_dist_thres_polygon 
        # self.obs_dist_hard_thres_polygon

        collision_polygons = [] # list of detected collisions' shapely Polygon or MultiPolygon objects
        collision_polygons_hard = [] # list of detected hard collisions' shapely Polygon or MultiPolygon objects

        # Check if the robot is colliding with the workspace 
        # (ie. is the robot obs thresh within the workspace_polygon)
          
        if not self.workspace_polygon.contains(self.obs_dist_thres_polygon):
            # contains: no points of self.obs_dist_thres_polygon lie in the exterior of self.workspace_polygon 
            # and at least one point of the interior of self.obs_dist_thres_polygon lies in the interior of self.workspace_polygon.
            
            # ie. Find the polygon at the exterior of the workspace_polygon
            collision_polygon = self.obs_dist_thres_polygon.difference(self.workspace_polygon)
            # collision_polygon_centeroid_point =  collision_polygon.centroid

            # if this collision polygon is also at the interior of the obs_dist_hard_thres_polygon, we need a special treat!
            if not self.workspace_polygon.contains(self.obs_dist_hard_thres_polygon):
                # ie. Find the polygon at the exterior of the workspace_polygon
                collision_polygon = self.obs_dist_hard_thres_polygon.difference(self.workspace_polygon)
                collision_polygons_hard.append(collision_polygon)
                # rospy.logwarn("Hard threshold for workspace is triggered")
                # rospy.logwarn(str(collision_polygon.geom_type))    
            else:
                collision_polygons.append(collision_polygon)
                # rospy.logwarn("Soft threshold for workspace is triggered")
                # rospy.logwarn(str(collision_polygon.geom_type))    

        # Check if the robot is colliding with any of the other robots
        for i in range(self.num_of_robots):      
            if i != self.index:
                rospy.logwarn(str(self.mobile_base_polygons[i].overlaps(self.obs_dist_thres_polygon)))      
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
                        collision_polygons_hard.append(collision_polygon)
                        rospy.logwarn("Hard threshold for hitting other robots is triggered")
                        rospy.logwarn(str(collision_polygon.geom_type))   
                    else:
                        collision_polygons.append(collision_polygon)
                        rospy.logwarn("Soft threshold for hitting other robots is triggered")
                        rospy.logwarn(str(collision_polygon.geom_type))   

        return collision_polygons, collision_polygons_hard
        # self.avoid_others(directions_to_avoid, avoiding_weights)


        

if __name__ == "__main__":
    collisionAvoidance2D = CollisionAvoidance2D()
    rospy.spin()
