/**
 * ROS2 Jazzy Stereo ORB-SLAM3 Node
 * Adapted from ORB-SLAM3: Examples/ROS/src/ros_stereo.cc
 */

#include "common.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace std;

class StereoSlamNode : public rclcpp::Node
{
public:
    StereoSlamNode() : Node("orb_slam3_stereo")
    {
        RCLCPP_INFO(this->get_logger(), "Initializing ORB-SLAM3 Stereo Node...");
        
        // Declare parameters
        this->declare_parameter<std::string>("voc_file", "file_not_set");
        this->declare_parameter<std::string>("settings_file", "file_not_set");
        this->declare_parameter<std::string>("world_frame_id", "map");
        this->declare_parameter<std::string>("cam_frame_id", "camera");
        this->declare_parameter<bool>("enable_pangolin", true);
        
        // Get parameters
        std::string voc_file = this->get_parameter("voc_file").as_string();
        std::string settings_file = this->get_parameter("settings_file").as_string();
        world_frame_id = this->get_parameter("world_frame_id").as_string();
        cam_frame_id = this->get_parameter("cam_frame_id").as_string();
        bool enable_pangolin = this->get_parameter("enable_pangolin").as_bool();
        
        // Validate parameters
        if (voc_file == "file_not_set" || settings_file == "file_not_set")
        {
            RCLCPP_ERROR(this->get_logger(), "Please provide voc_file and settings_file parameters");
            rclcpp::shutdown();
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Using vocabulary: %s", voc_file.c_str());
        RCLCPP_INFO(this->get_logger(), "Using settings: %s", settings_file.c_str());
        
        // Initialize ORB-SLAM3
        sensor_type = ORB_SLAM3::System::STEREO;
        pSLAM = new ORB_SLAM3::System(voc_file, settings_file, sensor_type, enable_pangolin);
        
        if (!pSLAM)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize ORB-SLAM3 system");
            rclcpp::shutdown();
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 system initialized successfully");
        
        // Create a timer to initialize after construction is complete
        init_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&StereoSlamNode::initializeAfterConstruction, this)
        );
    }
    
    void initializeAfterConstruction()
    {
        // Cancel the timer
        init_timer_->cancel();
        
        try {
            // Setup publishers and services (now safe to use shared_from_this())
            image_transport::ImageTransport it(shared_from_this());
            setup_publishers(shared_from_this(), it, this->get_name());
            setup_services(shared_from_this(), this->get_name());
            
            // Initialize synchronizer
            initializeSynchronizer();
            
            RCLCPP_INFO(this->get_logger(), "Stereo SLAM node ready. Waiting for synchronized images...");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception during initialization: %s", e.what());
            rclcpp::shutdown();
        }
    }
    
    ~StereoSlamNode()
    {
        if (pSLAM)
        {
            RCLCPP_INFO(this->get_logger(), "Shutting down ORB-SLAM3...");
            pSLAM->Shutdown();
            delete pSLAM;
            pSLAM = nullptr;
        }
    }

private:
    void initializeSynchronizer()
    {
        RCLCPP_INFO(this->get_logger(), "Setting up synchronizer...");
        
        // Create individual subscribers first for debugging
        left_sub_debug_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/left/image_raw", 10,
            std::bind(&StereoSlamNode::leftImageCallback, this, std::placeholders::_1));
        
        right_sub_debug_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/right/image_raw", 10, 
            std::bind(&StereoSlamNode::rightImageCallback, this, std::placeholders::_1));
        
        // Create message filter subscribers
        left_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
            this, "/left/image_raw");
        right_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
            this, "/right/image_raw");
        
        // Create synchronizer with more permissive settings
        typedef message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::Image, sensor_msgs::msg::Image> MySyncPolicy;
        
        sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(
            MySyncPolicy(20), *left_sub_, *right_sub_);  // Increased queue size
        
        // Register callback
        sync_->registerCallback(
            std::bind(&StereoSlamNode::stereoCallback, this, 
                     std::placeholders::_1, std::placeholders::_2));
        
        RCLCPP_INFO(this->get_logger(), "Subscribed to stereo topics:");
        RCLCPP_INFO(this->get_logger(), "  Left:  /left/image_raw");
        RCLCPP_INFO(this->get_logger(), "  Right: /right/image_raw");
        RCLCPP_INFO(this->get_logger(), "Synchronizer initialized with ApproximateTime policy");
    }
    
    // Debug callbacks to check individual image reception
    void leftImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        static int left_count = 0;
        left_count++;
        if (left_count % 30 == 0) {
            RCLCPP_INFO(this->get_logger(), "Received left image %d [%dx%d]", 
                       left_count, msg->width, msg->height);
        }
    }
    
    void rightImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        static int right_count = 0;
        right_count++;
        if (right_count % 30 == 0) {
            RCLCPP_INFO(this->get_logger(), "Received right image %d [%dx%d]", 
                       right_count, msg->width, msg->height);
        }
    }
    
    void stereoCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msgLeft,
                       const sensor_msgs::msg::Image::ConstSharedPtr& msgRight)
    {
        RCLCPP_INFO(this->get_logger(), "=== STEREO CALLBACK TRIGGERED ===");
        
        // Convert ROS time to rclcpp::Time
        rclcpp::Time msg_time = msgLeft->header.stamp;
        
        static int frame_count = 0;
        frame_count++;
        
        RCLCPP_INFO(this->get_logger(), "Processing stereo frame %d", frame_count);
        RCLCPP_INFO(this->get_logger(), "Left image: %dx%d, Right image: %dx%d", 
                   msgLeft->width, msgLeft->height, msgRight->width, msgRight->height);
        
        // Convert ROS images to OpenCV
        cv_bridge::CvImageConstPtr cv_ptrLeft, cv_ptrRight;
        try
        {
            cv_ptrLeft = cv_bridge::toCvShare(msgLeft, sensor_msgs::image_encodings::BGR8);
            cv_ptrRight = cv_bridge::toCvShare(msgRight, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        
        if (cv_ptrLeft->image.empty() || cv_ptrRight->image.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Received empty image(s)");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Both images valid, sending to ORB-SLAM3...");
        
        // Store current input image for tracking image fallback
        current_input_image_ = cv_ptrLeft->image.clone();
        
        try
        {
            // Convert rclcpp::Time to double (seconds)
            double timestamp = msg_time.seconds();
            
            RCLCPP_INFO(this->get_logger(), "Calling TrackStereo with timestamp: %.6f", timestamp);
            
            // ORB-SLAM3 stereo tracking
            Sophus::SE3f Tcw = pSLAM->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image, timestamp);
            
            RCLCPP_INFO(this->get_logger(), "TrackStereo completed");
            
            // Check if tracking was successful - cast the return value
            int state_int = pSLAM->GetTrackingState();
            ORB_SLAM3::Tracking::eTrackingState state = static_cast<ORB_SLAM3::Tracking::eTrackingState>(state_int);
            
            if (frame_count % 30 == 0) {
                switch(state) {
                    case ORB_SLAM3::Tracking::SYSTEM_NOT_READY:
                        RCLCPP_WARN(this->get_logger(), "SLAM System not ready");
                        break;
                    case ORB_SLAM3::Tracking::NO_IMAGES_YET:
                        RCLCPP_INFO(this->get_logger(), "Waiting for images");
                        break;
                    case ORB_SLAM3::Tracking::NOT_INITIALIZED:
                        RCLCPP_INFO(this->get_logger(), "Initializing SLAM...");
                        break;
                    case ORB_SLAM3::Tracking::OK:
                        RCLCPP_INFO(this->get_logger(), "Tracking OK");
                        break;
                    case ORB_SLAM3::Tracking::RECENTLY_LOST:
                        RCLCPP_WARN(this->get_logger(), "Tracking recently lost");
                        break;
                    case ORB_SLAM3::Tracking::LOST:
                        RCLCPP_WARN(this->get_logger(), "Tracking lost");
                        break;
                }
            }
            
            // Publish topics regardless of tracking state
            publish_topics(msg_time);
            
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Exception in stereo tracking: %s", e.what());
        }
        catch (...)
        {
            RCLCPP_ERROR(this->get_logger(), "Unknown exception in stereo tracking");
        }
    }
    
    // Member variables
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> left_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> right_sub_;
    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image>>> sync_;
    
    // Debug subscribers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_sub_debug_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_sub_debug_;
    
    // Store current input image locally
    cv::Mat current_input_image_;
    
    // Timer for delayed initialization
    rclcpp::TimerBase::SharedPtr init_timer_;
};

int main(int argc, char **argv)
{
    // Initialize ROS2
    rclcpp::init(argc, argv);
    
    if (argc > 1)
    {
        RCLCPP_WARN(rclcpp::get_logger("stereo_slam"), "Arguments supplied via command line are ignored.");
    }
    
    try
    {
        // Create and run the node
        auto node = std::make_shared<StereoSlamNode>();
        
        RCLCPP_INFO(node->get_logger(), "Starting ORB-SLAM3 Stereo Node...");
        RCLCPP_INFO(node->get_logger(), "Use Ctrl+C to stop");
        
        // Spin the node
        rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("stereo_slam"), "Exception in main: %s", e.what());
    }
    catch (...)
    {
        RCLCPP_ERROR(rclcpp::get_logger("stereo_slam"), "Unknown exception in main");
    }
    
    // Cleanup
    RCLCPP_INFO(rclcpp::get_logger("stereo_slam"), "Shutting down...");
    rclcpp::shutdown();
    return 0;
}