/**
 * ROS2 Jazzy Stereo ORB-SLAM3 Node - OPTIMIZED VERSION
 * Fixed for better FPS and tracking stability
 */

#include "common.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <thread>
#include <chrono>

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
        // this->declare_parameter<double>("sync_tolerance", 0.05);  // 50ms default
        this->declare_parameter<double>("sync_tolerance", 0.1);  // 50ms default
        this->declare_parameter<int>("skip_frames", 0);  // Skip frames for performance
        this->declare_parameter<bool>("enable_debug", false);  // Disable debug by default
        
        // Get parameters
        std::string voc_file = this->get_parameter("voc_file").as_string();
        std::string settings_file = this->get_parameter("settings_file").as_string();
        world_frame_id = this->get_parameter("world_frame_id").as_string();
        cam_frame_id = this->get_parameter("cam_frame_id").as_string();
        bool enable_pangolin = this->get_parameter("enable_pangolin").as_bool();
        sync_tolerance_ = this->get_parameter("sync_tolerance").as_double();
        skip_frames_ = this->get_parameter("skip_frames").as_int();
        enable_debug_ = this->get_parameter("enable_debug").as_bool();
        
        // Validate parameters
        if (voc_file == "file_not_set" || settings_file == "file_not_set")
        {
            RCLCPP_ERROR(this->get_logger(), "Please provide voc_file and settings_file parameters");
            rclcpp::shutdown();
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Using vocabulary: %s", voc_file.c_str());
        RCLCPP_INFO(this->get_logger(), "Using settings: %s", settings_file.c_str());
        RCLCPP_INFO(this->get_logger(), "Sync tolerance: %.1f ms", sync_tolerance_ * 1000);
        RCLCPP_INFO(this->get_logger(), "Frame skip: %d", skip_frames_);
        
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
        
        // Reset frame counter
        frame_count_ = 0;
        processed_count_ = 0;
        last_process_time_ = this->now();
        
        // Initialize after construction is complete
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
            // Setup publishers and services
            image_transport::ImageTransport it(shared_from_this());
            setup_publishers(shared_from_this(), it, this->get_name());
            setup_services(shared_from_this(), this->get_name());
            
            // Initialize synchronizer
            initializeSynchronizer();
            
            // Create FPS reporting timer
            fps_timer_ = this->create_wall_timer(
                std::chrono::seconds(2),
                std::bind(&StereoSlamNode::reportFPS, this)
            );
            
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
            
            // Save trajectories
            pSLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
            pSLAM->SaveTrajectoryTUM("CameraTrajectory.txt");
            
            delete pSLAM;
            pSLAM = nullptr;
        }
    }

private:
    void initializeSynchronizer()
    {
        RCLCPP_INFO(this->get_logger(), "Setting up synchronizer...");
        
        // Create message filter subscribers with sensor data QoS for better performance
        rmw_qos_profile_t custom_qos = rmw_qos_profile_sensor_data;
        custom_qos.depth = 1;  // Keep queue small for real-time performance
        
        left_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
            this, "/left/image_raw", custom_qos);
        right_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
            this, "/right/image_raw", custom_qos);
        
        // Create synchronizer with appropriate settings for USB cameras
        typedef message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::Image, sensor_msgs::msg::Image> MySyncPolicy;
        
        // Use smaller queue for better real-time performance
        sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(
            MySyncPolicy(10), *left_sub_, *right_sub_);
        
        // Register callback
        sync_->registerCallback(
            std::bind(&StereoSlamNode::stereoCallback, this, 
                     std::placeholders::_1, std::placeholders::_2));
        
        // Only create debug subscribers if debug is enabled
        if (enable_debug_) {
            left_sub_debug_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/left/image_raw", 1,
                std::bind(&StereoSlamNode::leftImageCallback, this, std::placeholders::_1));
            
            right_sub_debug_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/right/image_raw", 1, 
                std::bind(&StereoSlamNode::rightImageCallback, this, std::placeholders::_1));
        }
        
        RCLCPP_INFO(this->get_logger(), "Subscribed to stereo topics:");
        RCLCPP_INFO(this->get_logger(), "  Left:  /left/image_raw");
        RCLCPP_INFO(this->get_logger(), "  Right: /right/image_raw");
        RCLCPP_INFO(this->get_logger(), "Synchronizer initialized (queue=10, tolerance=%.1fms)", 
                    sync_tolerance_ * 1000);
    }
    
    // Debug callbacks (only if enabled)
    void leftImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        static int left_count = 0;
        left_count++;
        if (left_count % 100 == 0) {
            RCLCPP_DEBUG(this->get_logger(), "Left images: %d", left_count);
        }
    }
    
    void rightImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        static int right_count = 0;
        right_count++;
        if (right_count % 100 == 0) {
            RCLCPP_DEBUG(this->get_logger(), "Right images: %d", right_count);
        }
    }
    
    void stereoCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msgLeft,
                       const sensor_msgs::msg::Image::ConstSharedPtr& msgRight)
    {
        frame_count_++;
        
        // Skip frames if configured (for performance)
        if (skip_frames_ > 0 && (frame_count_ % (skip_frames_ + 1)) != 0) {
            return;
        }
        
        // Convert ROS time to rclcpp::Time
        rclcpp::Time msg_time = msgLeft->header.stamp;
        
        // Log first sync
        if (processed_count_ == 0) {
            RCLCPP_INFO(this->get_logger(), "First synchronized pair received!");
        }
        
        // Convert ROS images to OpenCV
        cv_bridge::CvImageConstPtr cv_ptrLeft, cv_ptrRight;
        try
        {
            // Use toCvShare for better performance (no copy)
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
        
        // Store current input image for tracking visualization
        current_input_image = cv_ptrLeft->image.clone();
        set_current_input_image(current_input_image);
        
        try
        {
            // Get timestamp in seconds
            double timestamp = msg_time.seconds();
            
            // ORB-SLAM3 stereo tracking
            auto start = std::chrono::high_resolution_clock::now();
            
            Sophus::SE3f Tcw = pSLAM->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image, timestamp);
            
            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
            
            processed_count_++;
            
            // Get tracking state
            int state_int = pSLAM->GetTrackingState();
            ORB_SLAM3::Tracking::eTrackingState state = 
                static_cast<ORB_SLAM3::Tracking::eTrackingState>(state_int);
            
            // Log tracking state periodically
            if (processed_count_ % 30 == 0) {
                const char* state_str = "UNKNOWN";
                switch(state) {
                    case ORB_SLAM3::Tracking::SYSTEM_NOT_READY:
                        state_str = "SYSTEM_NOT_READY"; break;
                    case ORB_SLAM3::Tracking::NO_IMAGES_YET:
                        state_str = "NO_IMAGES_YET"; break;
                    case ORB_SLAM3::Tracking::NOT_INITIALIZED:
                        state_str = "NOT_INITIALIZED"; break;
                    case ORB_SLAM3::Tracking::OK:
                        state_str = "OK"; break;
                    case ORB_SLAM3::Tracking::RECENTLY_LOST:
                        state_str = "RECENTLY_LOST"; break;
                    case ORB_SLAM3::Tracking::LOST:
                        state_str = "LOST"; break;
                }
                
                RCLCPP_INFO(this->get_logger(), 
                           "Frame %d | State: %s | Track time: %ld ms", 
                           processed_count_, state_str, duration.count());
                
                // If tracking is lost, provide hints
                if (state == ORB_SLAM3::Tracking::LOST || 
                    state == ORB_SLAM3::Tracking::RECENTLY_LOST) {
                    RCLCPP_WARN(this->get_logger(), 
                               "Tracking lost! Try: slower movement, better lighting, textured scenes");
                }
            }
            
            // Publish topics
            publish_topics(msg_time);
            
            // Update timing
            last_process_time_ = this->now();
            
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
    
    void reportFPS()
    {
        if (processed_count_ > 0) {
            auto duration = (this->now() - last_process_time_).seconds();
            if (duration < 5.0) {  // Only report if we've processed frames recently
                double fps = frame_count_ / 2.0;  // Approximate FPS over 2 seconds
                RCLCPP_INFO(this->get_logger(), 
                           "Performance: %.1f FPS | Processed: %d frames | Skipped: %d frames",
                           fps, processed_count_, frame_count_ - processed_count_);
            }
            frame_count_ = 0;  // Reset counter
        }
    }
    
    // Member variables
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> left_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> right_sub_;
    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image>>> sync_;
    
    // Debug subscribers (optional)
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_sub_debug_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_sub_debug_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr init_timer_;
    rclcpp::TimerBase::SharedPtr fps_timer_;
    
    // Performance tracking
    int frame_count_;
    int processed_count_;
    rclcpp::Time last_process_time_;
    
    // Parameters
    double sync_tolerance_;
    int skip_frames_;
    bool enable_debug_;
    
    // Store current input image
    cv::Mat current_input_image;
};

int main(int argc, char **argv)
{
    // Initialize ROS2
    rclcpp::init(argc, argv);
    
    // Set executor to SingleThreaded for better real-time performance
    rclcpp::executors::SingleThreadedExecutor executor;
    
    try
    {
        // Create the node
        auto node = std::make_shared<StereoSlamNode>();
        
        RCLCPP_INFO(node->get_logger(), "Starting ORB-SLAM3 Stereo Node...");
        RCLCPP_INFO(node->get_logger(), "Tips for better tracking:");
        RCLCPP_INFO(node->get_logger(), "  - Move cameras slowly, especially during initialization");
        RCLCPP_INFO(node->get_logger(), "  - Ensure good lighting and textured scenes");
        RCLCPP_INFO(node->get_logger(), "  - Avoid pure rotation, include translation for stereo");
        RCLCPP_INFO(node->get_logger(), "Use Ctrl+C to stop");
        
        // Add node to executor and spin
        executor.add_node(node);
        executor.spin();
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("stereo_slam"), "Exception in main: %s", e.what());
    }
    
    // Cleanup
    RCLCPP_INFO(rclcpp::get_logger("stereo_slam"), "Shutting down...");
    rclcpp::shutdown();
    return 0;
}