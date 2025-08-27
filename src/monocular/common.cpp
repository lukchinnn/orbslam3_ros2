#include "common.h"
#include "orbslam3_ros/srv/save_map.hpp"

// =========================
// Globals (definitions)
// =========================
ORB_SLAM3::System* pSLAM = nullptr;
ORB_SLAM3::System::eSensor sensor_type = ORB_SLAM3::System::NOT_SET;

std::string world_frame_id, cam_frame_id, imu_frame_id;

std::shared_ptr<rclcpp::Node> g_node;
std::unique_ptr<tf2_ros::TransformBroadcaster> g_tf_broadcaster;

rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr tracked_mappoints_pub;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr all_mappoints_pub;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr tracked_keypoints_pub;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr kf_markers_pub;
image_transport::Publisher tracking_img_pub;
// rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr tracking_img_pub;

// =========================
// Internals
// =========================
static inline bool has_nan(const Sophus::SE3f& T)
{
  const auto& t = T.translation();
  const auto R = T.rotationMatrix();
  return (!t.allFinite()) || (!R.allFinite());
}

static geometry_msgs::msg::TransformStamped SE3f_to_TransformStamped(const Sophus::SE3f& T,
                                                                     const std::string& parent,
                                                                     const std::string& child,
                                                                     const rclcpp::Time& stamp)
{
  geometry_msgs::msg::TransformStamped ts;
  ts.header.stamp = stamp;
  ts.header.frame_id = parent;
  ts.child_frame_id = child;

  const Eigen::Vector3f t = T.translation();
  const Eigen::Quaternionf q = T.unit_quaternion();

  ts.transform.translation.x = static_cast<double>(t.x());
  ts.transform.translation.y = static_cast<double>(t.y());
  ts.transform.translation.z = static_cast<double>(t.z());

  ts.transform.rotation.w = static_cast<double>(q.w());
  ts.transform.rotation.x = static_cast<double>(q.x());
  ts.transform.rotation.y = static_cast<double>(q.y());
  ts.transform.rotation.z = static_cast<double>(q.z());
  return ts;
}

// =========================
// Services
// =========================

static rclcpp::Service<orbslam3_ros::srv::SaveMap>::SharedPtr save_map_service;
static rclcpp::Service<orbslam3_ros::srv::SaveMap>::SharedPtr save_traj_service;

static void save_map_srv_cb(const std::shared_ptr<orbslam3_ros::srv::SaveMap::Request> req,
                            std::shared_ptr<orbslam3_ros::srv::SaveMap::Response> res)
{
  if (!pSLAM) {
    RCLCPP_ERROR(g_node->get_logger(), "pSLAM is null; cannot save map.");
    res->success = false;
    return;
  }
  res->success = pSLAM->SaveMap(req->name);
  if (res->success) {
    RCLCPP_INFO(g_node->get_logger(), "Map was saved as %s.osa", req->name.c_str());
  } else {
    RCLCPP_ERROR(g_node->get_logger(), "Map could not be saved.");
  }
}

static void save_traj_srv_cb(const std::shared_ptr<orbslam3_ros::srv::SaveMap::Request> req,
                             std::shared_ptr<orbslam3_ros::srv::SaveMap::Response> res)
{
  if (!pSLAM) {
    RCLCPP_ERROR(g_node->get_logger(), "pSLAM is null; cannot save trajectories.");
    res->success = false;
    return;
  }
  const std::string cam_traj_file = req->name + "_cam_traj.txt";
  const std::string kf_traj_file  = req->name + "_kf_traj.txt";
  try {
    pSLAM->SaveTrajectoryEuRoC(cam_traj_file);
    pSLAM->SaveKeyFrameTrajectoryEuRoC(kf_traj_file);
    res->success = true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(g_node->get_logger(), "Exception: %s", e.what());
    res->success = false;
  } catch (...) {
    RCLCPP_ERROR(g_node->get_logger(), "Unknown exception saving trajectories.");
    res->success = false;
  }
  if (!res->success) {
    RCLCPP_ERROR(g_node->get_logger(), "Estimated trajectory could not be saved.");
  }
}

// =========================
// Setup
// =========================
void setup_services(const std::shared_ptr<rclcpp::Node>& node, const std::string& node_name)
{
  g_node = node;
  if (!g_tf_broadcaster) {
    g_tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(g_node);
  }
  save_map_service = g_node->create_service<orbslam3_ros::srv::SaveMap>(
      node_name + "/save_map", &save_map_srv_cb);
  save_traj_service = g_node->create_service<orbslam3_ros::srv::SaveMap>(
      node_name + "/save_traj", &save_traj_srv_cb);
}

void setup_publishers(const std::shared_ptr<rclcpp::Node>& node,
                      image_transport::ImageTransport& it,
                      const std::string& node_name)
{
  g_node = node;
  if (!g_tf_broadcaster) {
    g_tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(g_node);
  }
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

  pose_pub              = g_node->create_publisher<geometry_msgs::msg::PoseStamped>(node_name + "/camera_pose", qos);
  tracked_mappoints_pub = g_node->create_publisher<sensor_msgs::msg::PointCloud2>(node_name + "/tracked_points", qos);
  tracked_keypoints_pub = g_node->create_publisher<sensor_msgs::msg::PointCloud2>(node_name + "/tracked_key_points", qos);
  all_mappoints_pub     = g_node->create_publisher<sensor_msgs::msg::PointCloud2>(node_name + "/all_points", qos);
  kf_markers_pub        = g_node->create_publisher<visualization_msgs::msg::Marker>(node_name + "/kf_markers",
                                                                                    rclcpp::QoS(10));
  // tracking_img_pub      = it.advertise(node_name + "/tracking_image");
  tracking_img_pub = it.advertise(node_name + "/tracking_image", 1);

  if (sensor_type == ORB_SLAM3::System::IMU_MONOCULAR ||
      sensor_type == ORB_SLAM3::System::IMU_STEREO    ||
      sensor_type == ORB_SLAM3::System::IMU_RGBD)
  {
    odom_pub = g_node->create_publisher<nav_msgs::msg::Odometry>(node_name + "/body_odom", qos);
  }
}
// void setup_publishers(const std::shared_ptr<rclcpp::Node>& node,
//                       const std::string& node_name)
// {
//   g_node = node;
//   if (!g_tf_broadcaster) {
//     g_tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(g_node);
//   }
//   auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

//   pose_pub              = g_node->create_publisher<geometry_msgs::msg::PoseStamped>(node_name + "/camera_pose", qos);
//   tracked_mappoints_pub = g_node->create_publisher<sensor_msgs::msg::PointCloud2>(node_name + "/tracked_points", qos);
//   tracked_keypoints_pub = g_node->create_publisher<sensor_msgs::msg::PointCloud2>(node_name + "/tracked_key_points", qos);
//   all_mappoints_pub     = g_node->create_publisher<sensor_msgs::msg::PointCloud2>(node_name + "/all_points", qos);
//   kf_markers_pub        = g_node->create_publisher<visualization_msgs::msg::Marker>(node_name + "/kf_markers", rclcpp::QoS(10));

//   // ✅ plain image publisher instead of image_transport
//   tracking_img_pub      = g_node->create_publisher<sensor_msgs::msg::Image>(node_name + "/tracking_image", qos);

//   if (sensor_type == ORB_SLAM3::System::IMU_MONOCULAR ||
//       sensor_type == ORB_SLAM3::System::IMU_STEREO    ||
//       sensor_type == ORB_SLAM3::System::IMU_RGBD)
//   {
//     odom_pub = g_node->create_publisher<nav_msgs::msg::Odometry>(node_name + "/body_odom", qos);
//   }
// }

// =========================
// Publish helpers
// =========================
void publish_camera_pose(const Sophus::SE3f& Twc, const rclcpp::Time& stamp)
{
  if (!pose_pub) return;
  geometry_msgs::msg::PoseStamped msg;
  msg.header.frame_id = world_frame_id;
  msg.header.stamp = stamp;

  const Eigen::Vector3f t = Twc.translation();
  const Eigen::Quaternionf q = Twc.unit_quaternion();
  msg.pose.position.x = t.x();
  msg.pose.position.y = t.y();
  msg.pose.position.z = t.z();
  msg.pose.orientation.w = q.w();
  msg.pose.orientation.x = q.x();
  msg.pose.orientation.y = q.y();
  msg.pose.orientation.z = q.z();

  pose_pub->publish(msg);
}

void publish_tf_transform(const Sophus::SE3f& T,
                          const std::string& parent,
                          const std::string& child,
                          const rclcpp::Time& stamp)
{
  if (!g_tf_broadcaster) return;
  g_tf_broadcaster->sendTransform(SE3f_to_TransformStamped(T, parent, child, stamp));
}

void publish_tracking_img(const cv::Mat& image, const rclcpp::Time& stamp)
{
  if (!tracking_img_pub) return;
  std_msgs::msg::Header header;
  header.stamp = stamp;
  header.frame_id = world_frame_id;
  auto msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
  tracking_img_pub.publish(msg);
}
// void publish_tracking_img(const cv::Mat& image, const rclcpp::Time& stamp)
// {
//   if (!tracking_img_pub) return;
//   std_msgs::msg::Header header;
//   header.stamp = stamp;
//   header.frame_id = world_frame_id;
//   auto msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
//   tracking_img_pub->publish(*msg);
// }

sensor_msgs::msg::PointCloud2 keypoints_to_pointcloud(const std::vector<cv::KeyPoint>& keypoints,
                                                      const rclcpp::Time& stamp)
{
  const int num_channels = 3; // x y z
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = stamp;
  cloud.header.frame_id = world_frame_id;
  cloud.height = 1;
  cloud.width  = static_cast<uint32_t>(keypoints.size());
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.point_step = num_channels * sizeof(float);
  cloud.row_step   = cloud.point_step * cloud.width;
  cloud.fields.resize(num_channels);

  const char* channel_id[] = {"x","y","z"};
  for (int i = 0; i < num_channels; ++i) {
    cloud.fields[i].name    = channel_id[i];
    cloud.fields[i].offset  = i * sizeof(float);
    cloud.fields[i].count   = 1;
    cloud.fields[i].datatype= sensor_msgs::msg::PointField::FLOAT32;
  }

  cloud.data.resize(cloud.row_step * cloud.height);
  auto* ptr = cloud.data.data();
  for (uint32_t i = 0; i < cloud.width; ++i) {
    const float data[3] = { keypoints[i].pt.x, keypoints[i].pt.y, 0.0f };
    std::memcpy(ptr + (i * cloud.point_step), data, sizeof(data));
  }
  return cloud;
}

sensor_msgs::msg::PointCloud2 mappoint_to_pointcloud(const std::vector<ORB_SLAM3::MapPoint*>& map_points,
                                                     const rclcpp::Time& stamp)
{
  const int num_channels = 3;
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = stamp;
  cloud.header.frame_id = world_frame_id;
  cloud.height = 1;
  cloud.width  = static_cast<uint32_t>(map_points.size());
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.point_step = num_channels * sizeof(float);
  cloud.row_step   = cloud.point_step * cloud.width;
  cloud.fields.resize(num_channels);

  const char* channel_id[] = {"x","y","z"};
  for (int i = 0; i < num_channels; ++i) {
    cloud.fields[i].name    = channel_id[i];
    cloud.fields[i].offset  = i * sizeof(float);
    cloud.fields[i].count   = 1;
    cloud.fields[i].datatype= sensor_msgs::msg::PointField::FLOAT32;
  }

  cloud.data.resize(cloud.row_step * cloud.height);
  auto* ptr = cloud.data.data();

  for (uint32_t i = 0; i < cloud.width; ++i) {
    const auto* mp = map_points[i];
    if (!mp) continue;
    // const Eigen::Vector3f P = mp->GetWorldPos(); // float
    // Eigen::Vector3f P = mp->GetWorldPos(); // float
    const Eigen::Vector3f P = const_cast<ORB_SLAM3::MapPoint*>(mp)->GetWorldPos();
    const float data[3] = { P.x(), P.y(), P.z() };
    std::memcpy(ptr + (i * cloud.point_step), data, sizeof(data));
  }
  return cloud;
}

void publish_keypoints(const std::vector<ORB_SLAM3::MapPoint*>& tracked_map_points,
                       const std::vector<cv::KeyPoint>& tracked_keypoints,
                       const rclcpp::Time& stamp)
{
  if (!tracked_keypoints_pub || tracked_keypoints.empty()) return;

  std::vector<cv::KeyPoint> filtered;
  filtered.reserve(tracked_keypoints.size());

  const size_t N = std::min(tracked_map_points.size(), tracked_keypoints.size());
  for (size_t i = 0; i < N; ++i) {
    if (tracked_map_points[i]) filtered.push_back(tracked_keypoints[i]);
  }
  auto cloud = keypoints_to_pointcloud(filtered, stamp);
  tracked_keypoints_pub->publish(cloud);
}

void publish_tracked_points(const std::vector<ORB_SLAM3::MapPoint*>& tracked_points,
                            const rclcpp::Time& stamp)
{
  if (!tracked_mappoints_pub) return;
  tracked_mappoints_pub->publish(mappoint_to_pointcloud(tracked_points, stamp));
}

void publish_all_points(const std::vector<ORB_SLAM3::MapPoint*>& map_points,
                        const rclcpp::Time& stamp)
{
  if (!all_mappoints_pub) return;
  all_mappoints_pub->publish(mappoint_to_pointcloud(map_points, stamp));
}

void publish_kf_markers(const std::vector<Sophus::SE3f>& vKFposes,
                        const rclcpp::Time& stamp)
{
  if (!kf_markers_pub) return;
  if (vKFposes.empty()) return;

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = world_frame_id;
  marker.header.stamp = stamp;
  marker.ns = "kf_markers";
  marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.lifetime = rclcpp::Duration(0, 0);
  marker.id = 0;

  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;
  marker.color.g = 1.0;
  marker.color.a = 1.0;

  marker.points.reserve(vKFposes.size());
  for (size_t i = 0; i < vKFposes.size(); ++i) {
    geometry_msgs::msg::Point p;
    p.x = vKFposes[i].translation().x();
    p.y = vKFposes[i].translation().y();
    p.z = vKFposes[i].translation().z();
    marker.points.push_back(p);
  }
  kf_markers_pub->publish(marker);
}

void publish_body_odom(const Sophus::SE3f& Twb,
                       const Eigen::Vector3f& Vwb,
                       const Eigen::Vector3f& Wbb,
                       const rclcpp::Time& stamp)
{
  if (!odom_pub) return;
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = world_frame_id;
  odom.child_frame_id  = imu_frame_id;
  odom.header.stamp    = stamp;

  const Eigen::Vector3f t = Twb.translation();
  const Eigen::Quaternionf q = Twb.unit_quaternion();

  odom.pose.pose.position.x = t.x();
  odom.pose.pose.position.y = t.y();
  odom.pose.pose.position.z = t.z();
  odom.pose.pose.orientation.w = q.w();
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();

  odom.twist.twist.linear.x  = Vwb.x();
  odom.twist.twist.linear.y  = Vwb.y();
  odom.twist.twist.linear.z  = Vwb.z();

  odom.twist.twist.angular.x = Wbb.x();
  odom.twist.twist.angular.y = Wbb.y();
  odom.twist.twist.angular.z = Wbb.z();

  odom_pub->publish(odom);
}

// =========================
// Main aggregator
// =========================
void publish_topics(const rclcpp::Time& msg_time, const Eigen::Vector3f& Wbb_body)
{
  if (!pSLAM) return;

  const Sophus::SE3f Twc = pSLAM->GetCamTwc();
  if (has_nan(Twc)) return;

  // Common topics
  publish_camera_pose(Twc, msg_time);
  publish_tf_transform(Twc, world_frame_id, cam_frame_id, msg_time);

  publish_tracking_img(pSLAM->GetCurrentFrame(), msg_time);

  publish_keypoints(pSLAM->GetTrackedMapPoints(),
                    pSLAM->GetTrackedKeyPoints(),
                    msg_time);

  publish_tracked_points(pSLAM->GetTrackedMapPoints(), msg_time);
  publish_all_points(pSLAM->GetAllMapPoints(), msg_time);
  publish_kf_markers(pSLAM->GetAllKeyframePoses(), msg_time);

  // IMU-specific
  if (sensor_type == ORB_SLAM3::System::IMU_MONOCULAR ||
      sensor_type == ORB_SLAM3::System::IMU_STEREO    ||
      sensor_type == ORB_SLAM3::System::IMU_RGBD)
  {
    const Sophus::SE3f Twb = pSLAM->GetImuTwb();
    const Eigen::Vector3f Vwb = pSLAM->GetImuVwb();

    publish_tf_transform(Twb, world_frame_id, imu_frame_id, msg_time);
    publish_body_odom(Twb, Vwb, Wbb_body, msg_time);
  }
}
