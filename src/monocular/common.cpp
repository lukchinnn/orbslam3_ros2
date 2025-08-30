#include "common.h"
#include "orbslam3_ros/srv/save_map.hpp"

// =========================
// Globals (definitions)
// =========================
ORB_SLAM3::System* pSLAM = nullptr;
ORB_SLAM3::System::eSensor sensor_type = ORB_SLAM3::System::NOT_SET;

std::string world_frame_id, cam_frame_id, imu_frame_id;

// Store current input image for tracking image publishing fallback
cv::Mat current_input_image;

std::shared_ptr<rclcpp::Node> g_node;
std::unique_ptr<tf2_ros::TransformBroadcaster> g_tf_broadcaster;

rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr tracked_mappoints_pub;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr all_mappoints_pub;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr tracked_keypoints_pub;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr kf_markers_pub;
image_transport::Publisher tracking_img_pub;

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
  
  RCLCPP_INFO(g_node->get_logger(), "Services setup complete");
}

void setup_publishers(const std::shared_ptr<rclcpp::Node>& node,
                      image_transport::ImageTransport& it,
                      const std::string& node_name)
{
    g_node = node;
    if (!g_tf_broadcaster) {
        g_tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(g_node);
        RCLCPP_INFO(g_node->get_logger(), "TF broadcaster initialized");
    }
    
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();  // Changed to reliable and increased queue

    // Create publishers with error checking
    pose_pub = g_node->create_publisher<geometry_msgs::msg::PoseStamped>(node_name + "/camera_pose", qos);
    if (!pose_pub) {
        RCLCPP_ERROR(g_node->get_logger(), "Failed to create pose publisher");
    } else {
        RCLCPP_INFO(g_node->get_logger(), "Created pose publisher: %s", (node_name + "/camera_pose").c_str());
    }
    
    tracked_mappoints_pub = g_node->create_publisher<sensor_msgs::msg::PointCloud2>(node_name + "/tracked_points", qos);
    if (tracked_mappoints_pub) {
        RCLCPP_INFO(g_node->get_logger(), "Created tracked points publisher");
    }
    
    tracked_keypoints_pub = g_node->create_publisher<sensor_msgs::msg::PointCloud2>(node_name + "/tracked_key_points", qos);
    if (tracked_keypoints_pub) {
        RCLCPP_INFO(g_node->get_logger(), "Created tracked keypoints publisher");
    }
    
    all_mappoints_pub = g_node->create_publisher<sensor_msgs::msg::PointCloud2>(node_name + "/all_points", qos);
    if (all_mappoints_pub) {
        RCLCPP_INFO(g_node->get_logger(), "Created all points publisher");
    }
    
    kf_markers_pub = g_node->create_publisher<visualization_msgs::msg::Marker>(node_name + "/kf_markers", rclcpp::QoS(10));
    if (kf_markers_pub) {
        RCLCPP_INFO(g_node->get_logger(), "Created keyframe markers publisher");
    }
    
    // Image transport publisher
    tracking_img_pub = it.advertise(node_name + "/tracking_image", 1);
    RCLCPP_INFO(g_node->get_logger(), "Created tracking image publisher");

    // IMU publishers (only if needed)
    if (sensor_type == ORB_SLAM3::System::IMU_MONOCULAR ||
        sensor_type == ORB_SLAM3::System::IMU_STEREO    ||
        sensor_type == ORB_SLAM3::System::IMU_RGBD)
    {
        odom_pub = g_node->create_publisher<nav_msgs::msg::Odometry>(node_name + "/body_odom", qos);
        if (odom_pub) {
            RCLCPP_INFO(g_node->get_logger(), "Created odometry publisher");
        }
    }
    
    RCLCPP_INFO(g_node->get_logger(), "All publishers setup complete");
}

// =========================
// Publish helpers
// =========================
void publish_camera_pose(const Sophus::SE3f& Twc, const rclcpp::Time& stamp)
{
    if (!pose_pub) {
        RCLCPP_WARN_THROTTLE(g_node->get_logger(), *g_node->get_clock(), 5000, "Pose publisher not initialized");
        return;
    }
    
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = world_frame_id;
    msg.header.stamp = stamp;

    const Eigen::Vector3f t = Twc.translation();
    const Eigen::Quaternionf q = Twc.unit_quaternion();
    
    msg.pose.position.x = static_cast<double>(t.x());
    msg.pose.position.y = static_cast<double>(t.y());
    msg.pose.position.z = static_cast<double>(t.z());
    msg.pose.orientation.w = static_cast<double>(q.w());
    msg.pose.orientation.x = static_cast<double>(q.x());
    msg.pose.orientation.y = static_cast<double>(q.y());
    msg.pose.orientation.z = static_cast<double>(q.z());

    pose_pub->publish(msg);
    
    // Debug output (throttled)
    static int pose_count = 0;
    if (++pose_count % 60 == 0) {
        RCLCPP_INFO(g_node->get_logger(), "Published pose %d: [%.3f, %.3f, %.3f]", 
                   pose_count, t.x(), t.y(), t.z());
    }
}

void publish_tf_transform(const Sophus::SE3f& T,
                          const std::string& parent,
                          const std::string& child,
                          const rclcpp::Time& stamp)
{
    if (!g_tf_broadcaster) {
        RCLCPP_WARN_THROTTLE(g_node->get_logger(), *g_node->get_clock(), 5000, "TF broadcaster not initialized");
        return;
    }
    
    if (has_nan(T)) {
        RCLCPP_WARN_THROTTLE(g_node->get_logger(), *g_node->get_clock(), 2000, "Transform has NaN values, skipping");
        return;
    }
    
    auto transform = SE3f_to_TransformStamped(T, parent, child, stamp);
    g_tf_broadcaster->sendTransform(transform);
    
    // Debug output (throttled)
    static int tf_count = 0;
    if (++tf_count % 60 == 0) {
        RCLCPP_INFO(g_node->get_logger(), "Published TF %d: %s -> %s", tf_count, parent.c_str(), child.c_str());
    }
}

void publish_tracking_img(const cv::Mat& image, const rclcpp::Time& stamp)
{
    if (!tracking_img_pub) {
        RCLCPP_WARN_THROTTLE(g_node->get_logger(), *g_node->get_clock(), 5000, "Tracking image publisher not initialized");
        return;
    }
    
    if (image.empty()) {
        RCLCPP_WARN_THROTTLE(g_node->get_logger(), *g_node->get_clock(), 2000, "Empty tracking image received");
        return;
    }
    
    try {
        // Ensure image is in correct format
        cv::Mat img_to_publish;
        if (image.channels() == 1) {
            // Convert grayscale to BGR
            cv::cvtColor(image, img_to_publish, cv::COLOR_GRAY2BGR);
        } else if (image.channels() == 3) {
            img_to_publish = image.clone();
        } else {
            RCLCPP_WARN_THROTTLE(g_node->get_logger(), *g_node->get_clock(), 2000, 
                                 "Unsupported image format: %d channels", image.channels());
            return;
        }
        
        std_msgs::msg::Header header;
        header.stamp = stamp;
        header.frame_id = cam_frame_id;
        
        // Create and publish the message
        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(header, "bgr8", img_to_publish).toImageMsg();
        tracking_img_pub.publish(*msg);
        
        // Debug output (throttled)
        static int img_count = 0;
        if (++img_count % 60 == 0) {
            RCLCPP_INFO(g_node->get_logger(), "Published tracking image %d [%dx%d, %d channels]", 
                       img_count, img_to_publish.cols, img_to_publish.rows, img_to_publish.channels());
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR_THROTTLE(g_node->get_logger(), *g_node->get_clock(), 5000, 
                              "Exception in publish_tracking_img: %s", e.what());
    }
}

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
    // Filter out null pointers first
    std::vector<ORB_SLAM3::MapPoint*> valid_points;
    for (auto* mp : map_points) {
        if (mp && !mp->isBad()) {  // Check if point exists and is not bad
            valid_points.push_back(mp);
        }
    }
    
    const int num_channels = 3;
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = world_frame_id;
    cloud.height = 1;
    cloud.width  = static_cast<uint32_t>(valid_points.size());
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
        try {
            const Eigen::Vector3f P = valid_points[i]->GetWorldPos();
            const float data[3] = { P.x(), P.y(), P.z() };
            std::memcpy(ptr + (i * cloud.point_step), data, sizeof(data));
        } catch (const std::exception& e) {
            // If we can't get position, use zero
            const float data[3] = { 0.0f, 0.0f, 0.0f };
            std::memcpy(ptr + (i * cloud.point_step), data, sizeof(data));
        }
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
        if (tracked_map_points[i] && !tracked_map_points[i]->isBad()) {
            filtered.push_back(tracked_keypoints[i]);
        }
    }
    
    if (!filtered.empty()) {
        auto cloud = keypoints_to_pointcloud(filtered, stamp);
        tracked_keypoints_pub->publish(cloud);
    }
}

void publish_tracked_points(const std::vector<ORB_SLAM3::MapPoint*>& tracked_points,
                            const rclcpp::Time& stamp)
{
    if (!tracked_mappoints_pub || tracked_points.empty()) return;
    auto cloud = mappoint_to_pointcloud(tracked_points, stamp);
    tracked_mappoints_pub->publish(cloud);
}

void publish_all_points(const std::vector<ORB_SLAM3::MapPoint*>& map_points,
                        const rclcpp::Time& stamp)
{
    if (!all_mappoints_pub || map_points.empty()) return;
    auto cloud = mappoint_to_pointcloud(map_points, stamp);
    all_mappoints_pub->publish(cloud);
}

void publish_kf_markers(const std::vector<Sophus::SE3f>& vKFposes,
                        const rclcpp::Time& stamp)
{
    if (!kf_markers_pub || vKFposes.empty()) return;

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
        p.x = static_cast<double>(vKFposes[i].translation().x());
        p.y = static_cast<double>(vKFposes[i].translation().y());
        p.z = static_cast<double>(vKFposes[i].translation().z());
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

    odom.pose.pose.position.x = static_cast<double>(t.x());
    odom.pose.pose.position.y = static_cast<double>(t.y());
    odom.pose.pose.position.z = static_cast<double>(t.z());
    odom.pose.pose.orientation.w = static_cast<double>(q.w());
    odom.pose.pose.orientation.x = static_cast<double>(q.x());
    odom.pose.pose.orientation.y = static_cast<double>(q.y());
    odom.pose.pose.orientation.z = static_cast<double>(q.z());

    odom.twist.twist.linear.x  = static_cast<double>(Vwb.x());
    odom.twist.twist.linear.y  = static_cast<double>(Vwb.y());
    odom.twist.twist.linear.z  = static_cast<double>(Vwb.z());

    odom.twist.twist.angular.x = static_cast<double>(Wbb.x());
    odom.twist.twist.angular.y = static_cast<double>(Wbb.y());
    odom.twist.twist.angular.z = static_cast<double>(Wbb.z());

    odom_pub->publish(odom);
}

// =========================
// Main aggregator
// =========================
void publish_topics(const rclcpp::Time& msg_time, const Eigen::Vector3f& Wbb_body)
{
    if (!pSLAM) {
        RCLCPP_WARN_THROTTLE(g_node->get_logger(), *g_node->get_clock(), 5000, "pSLAM is null in publish_topics");
        return;
    }

    static int publish_count = 0;
    publish_count++;

    try {
        // Get current camera pose
        const Sophus::SE3f Twc = pSLAM->GetCamTwc();
        if (has_nan(Twc)) {
            RCLCPP_WARN_THROTTLE(g_node->get_logger(), *g_node->get_clock(), 2000, "Camera pose has NaN values");
            return;
        }

        // Always publish camera pose and TF
        publish_camera_pose(Twc, msg_time);
        publish_tf_transform(Twc, world_frame_id, cam_frame_id, msg_time);

        // Try to publish tracking image - with multiple fallback methods
        try {
            cv::Mat tracking_img;
            
            // Method 1: Try to get the annotated tracking frame
            try {
                tracking_img = pSLAM->GetCurrentFrame();
            } catch (...) {
                // Method doesn't exist, try alternatives
            }
            
            // Method 2: If that fails, try alternative ORB-SLAM3 methods
            if (tracking_img.empty()) {
                try {
                    // Try different possible method names
                    tracking_img = pSLAM->GetTrackingState();  
                } catch (...) {
                    // Method doesn't exist
                }
            }
            
            // Method 3: Use the stored input image as fallback
            if (tracking_img.empty() && !current_input_image.empty()) {
                RCLCPP_INFO_THROTTLE(g_node->get_logger(), *g_node->get_clock(), 10000, 
                                     "Using input image as tracking image fallback");
                tracking_img = current_input_image.clone();
                
                // Add some basic annotation if possible
                if (tracking_img.channels() == 1) {
                    cv::cvtColor(tracking_img, tracking_img, cv::COLOR_GRAY2BGR);
                }
                
                // Add text overlay to indicate this is a fallback
                cv::putText(tracking_img, "Input Image", cv::Point(10, 30), 
                           cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
            }
            
            if (!tracking_img.empty()) {
                publish_tracking_img(tracking_img, msg_time);
            } else {
                RCLCPP_WARN_THROTTLE(g_node->get_logger(), *g_node->get_clock(), 10000, 
                                     "No tracking image available from any source");
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(g_node->get_logger(), *g_node->get_clock(), 5000, "Exception getting tracking image: %s", e.what());
        }

        // Get tracked map points and keypoints
        try {
            std::vector<ORB_SLAM3::MapPoint*> tracked_map_points = pSLAM->GetTrackedMapPoints();
            std::vector<cv::KeyPoint> tracked_keypoints = pSLAM->GetTrackedKeyPoints();
            
            if (!tracked_map_points.empty() && !tracked_keypoints.empty()) {
                publish_keypoints(tracked_map_points, tracked_keypoints, msg_time);
                publish_tracked_points(tracked_map_points, msg_time);
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(g_node->get_logger(), *g_node->get_clock(), 5000, "Exception getting tracked points: %s", e.what());
        }

        // Get all map points
        try {
            std::vector<ORB_SLAM3::MapPoint*> all_map_points = pSLAM->GetAllMapPoints();
            if (!all_map_points.empty()) {
                publish_all_points(all_map_points, msg_time);
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(g_node->get_logger(), *g_node->get_clock(), 5000, "Exception getting all points: %s", e.what());
        }

        // Get keyframe poses
        try {
            std::vector<Sophus::SE3f> kf_poses = pSLAM->GetAllKeyframePoses();
            if (!kf_poses.empty()) {
                publish_kf_markers(kf_poses, msg_time);
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(g_node->get_logger(), *g_node->get_clock(), 5000, "Exception getting keyframes: %s", e.what());
        }

        // IMU-specific publishing
        if (sensor_type == ORB_SLAM3::System::IMU_MONOCULAR ||
            sensor_type == ORB_SLAM3::System::IMU_STEREO    ||
            sensor_type == ORB_SLAM3::System::IMU_RGBD)
        {
            try {
                const Sophus::SE3f Twb = pSLAM->GetImuTwb();
                const Eigen::Vector3f Vwb = pSLAM->GetImuVwb();

                if (!has_nan(Twb)) {
                    publish_tf_transform(Twb, world_frame_id, imu_frame_id, msg_time);
                    publish_body_odom(Twb, Vwb, Wbb_body, msg_time);
                }
            } catch (const std::exception& e) {
                RCLCPP_WARN_THROTTLE(g_node->get_logger(), *g_node->get_clock(), 5000, "Exception in IMU publishing: %s", e.what());
            }
        }

        // Debug output (every 60 publishes)
        if (publish_count % 60 == 0) {
            RCLCPP_INFO(g_node->get_logger(), "Published topics %d times", publish_count);
        }

    } catch (const std::exception& e) {
        RCLCPP_ERROR_THROTTLE(g_node->get_logger(), *g_node->get_clock(), 5000, "Exception in publish_topics: %s", e.what());
    }
}