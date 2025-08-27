#pragma once

#include <string>
#include <vector>
#include <memory>
#include <cstring>

#include <Eigen/Core>
#include <sophus/se3.hpp>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/header.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <tf2_ros/transform_broadcaster.h>

// ORB-SLAM3
#include <System.h>
#include <ImuTypes.h>

// === Globals (kept for parity with your original code) ===
extern ORB_SLAM3::System* pSLAM;
extern ORB_SLAM3::System::eSensor sensor_type;

extern std::string world_frame_id, cam_frame_id, imu_frame_id;

// Node-scoped helpers (created inside setup_*)
extern std::shared_ptr<rclcpp::Node> g_node;
extern std::unique_ptr<tf2_ros::TransformBroadcaster> g_tf_broadcaster;

// Publishers
extern rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
extern rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
extern rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr tracked_mappoints_pub;
extern rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr all_mappoints_pub;
extern rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr tracked_keypoints_pub;
extern rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr kf_markers_pub;
extern image_transport::Publisher tracking_img_pub;
// extern rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr tracking_img_pub;

// === Setup ===
void setup_services(const std::shared_ptr<rclcpp::Node>& node, const std::string& node_name);
void setup_publishers(const std::shared_ptr<rclcpp::Node>& node,
                      image_transport::ImageTransport& it,
                      const std::string& node_name);
// void setup_publishers(const std::shared_ptr<rclcpp::Node>& node,
//                       const std::string& node_name);

// === High-level publish entry ===
void publish_topics(const rclcpp::Time& msg_time,
                    const Eigen::Vector3f& Wbb_body = Eigen::Vector3f::Zero());

// === Individual publishers ===
void publish_camera_pose(const Sophus::SE3f& Twc, const rclcpp::Time& stamp);
void publish_tf_transform(const Sophus::SE3f& T,
                          const std::string& parent,
                          const std::string& child,
                          const rclcpp::Time& stamp);
void publish_tracking_img(const cv::Mat& image, const rclcpp::Time& stamp);
void publish_tracked_points(const std::vector<ORB_SLAM3::MapPoint*>& tracked_points,
                            const rclcpp::Time& stamp);
void publish_all_points(const std::vector<ORB_SLAM3::MapPoint*>& map_points,
                        const rclcpp::Time& stamp);
void publish_keypoints(const std::vector<ORB_SLAM3::MapPoint*>& tracked_map_points,
                       const std::vector<cv::KeyPoint>& tracked_keypoints,
                       const rclcpp::Time& stamp);
void publish_kf_markers(const std::vector<Sophus::SE3f>& vKFposes,
                        const rclcpp::Time& stamp);
void publish_body_odom(const Sophus::SE3f& Twb,
                       const Eigen::Vector3f& Vwb,
                       const Eigen::Vector3f& Wbb,
                       const rclcpp::Time& stamp);

// === Converters ===
sensor_msgs::msg::PointCloud2 keypoints_to_pointcloud(const std::vector<cv::KeyPoint>& keypoints,
                                                      const rclcpp::Time& stamp);
sensor_msgs::msg::PointCloud2 mappoint_to_pointcloud(const std::vector<ORB_SLAM3::MapPoint*>& map_points,
                                                     const rclcpp::Time& stamp);
