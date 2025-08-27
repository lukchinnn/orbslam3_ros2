/**
 * ROS 2 Jazzy monocular bridge (debug version) - FIXED
 */
#include "/home/lukchin/skuba_ws/src/orbslam3_ros2/include/common.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>

class MonoNode : public rclcpp::Node
{
public:
    MonoNode() : rclcpp::Node("mono"), pSLAM(nullptr)
    {
        // Parameters
        this->declare_parameter<std::string>("voc_file", "file_not_set");
        this->declare_parameter<std::string>("settings_file", "file_not_set");
        this->declare_parameter<std::string>("world_frame_id", "map");
        this->declare_parameter<std::string>("cam_frame_id", "camera_link");
        this->declare_parameter<bool>("enable_pangolin", true);
        this->declare_parameter<std::string>("image_topic", "/camera/camera/color/image_raw");
        
        const std::string voc_file = this->get_parameter("voc_file").as_string();
        const std::string settings_file = this->get_parameter("settings_file").as_string();
        world_frame_id = this->get_parameter("world_frame_id").as_string();
        cam_frame_id = this->get_parameter("cam_frame_id").as_string();
        const bool enable_pangolin = this->get_parameter("enable_pangolin").as_bool();
        image_topic_ = this->get_parameter("image_topic").as_string();
        
        std::cout << "This is image topic: " << image_topic_ << std::endl;
        std::cout << "World frame: " << world_frame_id << ", Camera frame: " << cam_frame_id << std::endl;
        
        if (voc_file == "file_not_set" || settings_file == "file_not_set") {
            RCLCPP_ERROR(this->get_logger(), "Please provide voc_file and settings_file in the launch file");
            throw std::runtime_error("Missing voc/settings file parameters");
        }
        
        // SET GLOBAL VARIABLES BEFORE INITIALIZING SLAM
        ::world_frame_id = world_frame_id;
        ::cam_frame_id = cam_frame_id;
        ::sensor_type = ORB_SLAM3::System::MONOCULAR;
        
        // Initialize ORB-SLAM3 (Monocular)
        sensor_type = ORB_SLAM3::System::MONOCULAR;
        pSLAM = new ORB_SLAM3::System(voc_file, settings_file, sensor_type, enable_pangolin);
        
        // SET GLOBAL pSLAM POINTER
        ::pSLAM = pSLAM;
        
        std::cout << "ORB-SLAM3 initialized successfully" << std::endl;
    }
    
    void start()
    {
        std::cout << "Starting node setup..." << std::endl;
        
        // SETUP PUBLISHERS FIRST, BEFORE SUBSCRIBER
        try {
            auto it = std::make_shared<image_transport::ImageTransport>(shared_from_this());
            setup_publishers(shared_from_this(), *it, this->get_name());
            setup_services(shared_from_this(), this->get_name());
            std::cout << "Publishers and services setup complete" << std::endl;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to setup publishers: %s", e.what());
            throw;
        }
        
        // Image subscriber - use the parameter topic
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            image_topic_,
            rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
            std::bind(&MonoNode::grabImage, this, std::placeholders::_1)
        );
        
        RCLCPP_INFO(this->get_logger(), "Subscribed to image topic: %s", image_topic_.c_str());
        std::cout << "Node setup complete" << std::endl;
    }
    
    ~MonoNode() override
    {
        if (pSLAM) {
            pSLAM->Shutdown();
            delete pSLAM;
            pSLAM = nullptr;
            ::pSLAM = nullptr; // Clear global pointer
        }
    }

private:
    void grabImage(const sensor_msgs::msg::Image::ConstSharedPtr msg)
    {
        static int frame_count = 0;
        frame_count++;
        
        if (!pSLAM) {
            RCLCPP_WARN(this->get_logger(), "SLAM system not initialized");
            return;
        }
        
        // Debug: Log every 30 frames
        if (frame_count % 30 == 0) {
            RCLCPP_INFO(this->get_logger(), "Processing frame %d", frame_count);
        }
        
        // Convert to cv::Mat
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(msg);
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        
        // Check image validity
        if (cv_ptr->image.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty image");
            return;
        }
        
        const double tsec = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        
        // Track with ORB-SLAM3
        try {
            Sophus::SE3f pose = pSLAM->TrackMonocular(cv_ptr->image, tsec);
            
            // Get tracking state
            int tracking_state = pSLAM->GetTrackingState();
            
            // Debug: Check if SLAM is tracking (every 30 frames)
            if (frame_count % 30 == 0) {
                if (tracking_state == ORB_SLAM3::Tracking::OK) {
                    RCLCPP_INFO(this->get_logger(), "SLAM tracking OK - Frame %d", frame_count);
                } else {
                    RCLCPP_WARN(this->get_logger(), "SLAM tracking state: %d - Frame %d", tracking_state, frame_count);
                }
            }
            
            // Always call publish_topics, even if not tracking perfectly
            // This allows us to see what's happening
            publish_topics(msg->header.stamp, Eigen::Vector3f::Zero());
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in TrackMonocular: %s", e.what());
        }
    }
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    std::string world_frame_id;
    std::string cam_frame_id;
    std::string image_topic_;
    ORB_SLAM3::System* pSLAM;
    ORB_SLAM3::System::eSensor sensor_type;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<MonoNode>();
        node->start();
        
        RCLCPP_INFO(rclcpp::get_logger("mono_main"), "Starting to spin...");
        rclcpp::spin(node);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("mono_main"), "Exception: %s", e.what());
        return -1;
    }
    
    rclcpp::shutdown();
    return 0;
}