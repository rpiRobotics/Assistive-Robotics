#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <cmath>
#include <map>
#include <string>
#include <vector>
#include <algorithm>
#include <cctype>

// Helper function to convert strings to lowercase
std::string toLower(const std::string& str) {
    std::string lower_str = str;
    std::transform(lower_str.begin(), lower_str.end(), lower_str.begin(),
                   [](unsigned char c){ return std::tolower(c); });
    return lower_str;
}

// Define the Kinect body joint dictionary
std::map<std::string, int> KINECT_JOINT_DICT = {
    {"JOINT_PELVIS", 0}, 
    {"JOINT_SPINE_NAVEL", 1}, 
    {"JOINT_SPINE_CHEST", 2}, 
    {"JOINT_NECK", 3},
    {"JOINT_CLAVICLE_LEFT", 4}, 
    {"JOINT_SHOULDER_LEFT", 5}, 
    {"JOINT_ELBOW_LEFT", 6}, 
    {"JOINT_WRIST_LEFT", 7},
    {"JOINT_HAND_LEFT", 8},
    {"JOINT_HANDTIP_LEFT", 9}, 
    {"JOINT_THUMB_LEFT", 10}, 
    {"JOINT_CLAVICLE_RIGHT", 11},
    {"JOINT_SHOULDER_RIGHT", 12}, 
    {"JOINT_ELBOW_RIGHT", 13}, 
    {"JOINT_WRIST_RIGHT", 14}, 
    {"JOINT_HAND_RIGHT", 15},
    {"JOINT_HANDTIP_RIGHT", 16}, 
    {"JOINT_THUMB_RIGHT", 17}, 
    {"JOINT_HIP_LEFT", 18}, 
    {"JOINT_KNEE_LEFT", 19},
    {"JOINT_ANKLE_LEFT", 20}, 
    {"JOINT_FOOT_LEFT", 21}, 
    {"JOINT_HIP_RIGHT", 22}, 
    {"JOINT_KNEE_RIGHT", 23},
    {"JOINT_ANKLE_RIGHT", 24}, 
    {"JOINT_FOOT_RIGHT", 25}, 
    {"JOINT_HEAD", 26}, 
    {"JOINT_NOSE", 27},
    {"JOINT_EYE_LEFT", 28}, 
    {"JOINT_EAR_LEFT", 29}, 
    {"JOINT_EYE_RIGHT", 30}, 
    {"JOINT_EAR_RIGHT", 31}
};

class Kinect2BodyAllJointsTf {
public:
    Kinect2BodyAllJointsTf(ros::NodeHandle &nh, ros::NodeHandle &nh_local): 
        nh_(nh), 
        nh_local_(nh_local)
    {
        nh_local_.param<std::string>("kinect_body_tracking_data_topic_name", body_tracking_topic_, "/body_tracking_data");
        nh_local_.param<std::string>("tf_camera_frame_id", camera_frame_id_, "depth_camera_link");
        nh_local_.param<std::string>("body_joints_tf_prefix", joints_tf_prefix_, "");

        // Create covariance vector with size 36 = 6x6 for (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
        nh_local_.param<std::vector<double>>("pose_covariance_diagonal_max", covariance_max_, {1., 1., 1., 1., 1., 1.});
        nh_local_.param<std::vector<double>>("pose_covariance_diagonal_min", covariance_min_, {0.1, 0.1, 0.1, 0.1, 0.1, 0.1});
        covariance_.resize(36, 0.0); // Initialize the covariance vector with zeros

        nh_local_.param<bool>("tf_broadcast_enable", tf_broadcast_enable_, true);

        // Get reliability function parameters m and s
        nh_local_.param("reliability_func_param_m", m_, 0.0);
        nh_local_.param("reliability_func_param_s", s_, 0.42);

        // Initialize pose publishers for each joint
        for (const auto& joint : KINECT_JOINT_DICT) {
            std::string topic_name = "Pose_" + joints_tf_prefix_ + toLower(joint.first);
            ros::Publisher pub = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(topic_name, 2);
            pose_publisher_map_[joint.first] = pub;
        }

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>();
        body_tracking_sub_ = nh_.subscribe(body_tracking_topic_, 1, &Kinect2BodyAllJointsTf::bodyTrackingCallback, this);
    }

    void bodyTrackingCallback(const visualization_msgs::MarkerArray& msg) {
        if (msg.markers.empty()) {
            ROS_ERROR_THROTTLE(5, "No body detected, waiting to detect...");
            return;
        }

        // If at least one body is detected 
        for (const auto& joint : KINECT_JOINT_DICT) {
            const auto& marker = msg.markers[joint.second];

            geometry_msgs::TransformStamped t;
            t.header.stamp = marker.header.stamp;
            t.header.frame_id = camera_frame_id_;
            t.child_frame_id = joints_tf_prefix_ + toLower(joint.first); // Lower case the joint name that is only joint.first

            t.transform.translation.x = marker.pose.position.x;
            t.transform.translation.y = marker.pose.position.y;
            t.transform.translation.z = marker.pose.position.z;
            t.transform.rotation = marker.pose.orientation;
            
            if (tf_broadcast_enable_){
                tf_broadcaster_->sendTransform(t);
            }

            // Publish pose with covariance of that body joint
            // calculate its distance to camera
            double distance = std::sqrt(std::pow(marker.pose.position.x, 2) +
                                        std::pow(marker.pose.position.y, 2) +
                                        std::pow(marker.pose.position.z, 2));
            // calculate reliability score corresponding to that distance
            double reliability_score = calculateReliability(distance);
            // calculate corresponding covariance based on the reliability score
            calculateCovariance(reliability_score);

            // Create the PoseWithCovarianceStamped msg
            geometry_msgs::PoseWithCovarianceStamped pose_msg;
            pose_msg.header = t.header;

            pose_msg.pose.pose = marker.pose;
            // Assign the covariance to the pose
            std::copy(covariance_.begin(), covariance_.end(), pose_msg.pose.covariance.begin());
            
            // Publish PoseWithCovarianceStamped
            pose_publisher_map_[joint.first].publish(pose_msg);
        }
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_local_;

    std::string body_tracking_topic_;
    std::string camera_frame_id_;
    std::string joints_tf_prefix_;

    ros::Subscriber body_tracking_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::map<std::string, ros::Publisher> pose_publisher_map_;
    // std::map<std::string, ros::Publisher> pose_publishers_;
    
    double m_, s_;
    std::vector<double> covariance_max_;
    std::vector<double> covariance_min_;
    std::vector<double> covariance_;
    bool tf_broadcast_enable_;

    double calculateReliability(const double& distance) {
        // returns score between 0 - 1. See https://www.desmos.com/calculator/dprpp1kcu0 for details.
        // 1: max reliable, 0: min reliable.

        double log_normal = (1.0 / (distance * s_ * sqrt(2 * M_PI))) * exp(-((log(distance) - m_) * (log(distance) - m_)) / (2 * s_ * s_));
        double k = (1.0 / (s_ * sqrt(2 * M_PI))) * exp((s_ * s_ - 2 * m_) / 2.0); // normalization factor 
        return log_normal / k; // score
    }

    void calculateCovariance(const double& reliability_score) {
        for (int i = 0; i < 6; ++i) {
            covariance_[i * 7] = interpolateLinear(reliability_score, covariance_min_[i], covariance_max_[i]);
        }
    }

    double interpolateLinear(const double& score, const double& min, const double& max) {
        return min + (1.0 - score) * (max - min);
    }

};

int main(int argc, char** argv) {
    // ros::init(argc, argv, "dlo_simulator_node", ros::init_options::AnonymousName);
    ros::init(argc, argv, "tf_camera_body_all_joints_broadcaster");
    
    ros::NodeHandle nh("");
    ros::NodeHandle nh_local("~");

    Kinect2BodyAllJointsTf broadcaster(nh, nh_local);
    ros::spin();
    return 0;
}
