/**
 * ROS 2 Jazzy monocular bridge (debug version)
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
        this->declare_parameter<std::string>("cam_frame_id", "camera");
        this->declare_parameter<bool>("enable_pangolin", true);
        this->declare_parameter<std::string>("image_topic", "/camera/camera/color/image_raw");
        
        const std::string voc_file = this->get_parameter("voc_file").as_string();
        const std::string settings_file = this->get_parameter("settings_file").as_string();
        world_frame_id = this->get_parameter("world_frame_id").as_string();
        cam_frame_id = this->get_parameter("cam_frame_id").as_string();
        const bool enable_pangolin = this->get_parameter("enable_pangolin").as_bool();
        const std::string image_topic = this->get_parameter("image_topic").as_string();
        
        std::cout << "This is image topic: " << image_topic << std::endl;
        
        if (voc_file == "file_not_set" || settings_file == "file_not_set") {
            RCLCPP_ERROR(this->get_logger(), "Please provide voc_file and settings_file in the launch file");
            throw std::runtime_error("Missing voc/settings file parameters");
        }
        
        // Initialize ORB-SLAM3 (Monocular)
        sensor_type = ORB_SLAM3::System::MONOCULAR;
        pSLAM = new ORB_SLAM3::System(voc_file, settings_file, sensor_type, enable_pangolin);
        
        std::cout << "ORB-SLAM3 initialized successfully" << std::endl;
    }
    
    void start()
    {
        std::cout << "Starting node setup..." << std::endl;
        
        // Create image transport using regular subscription instead
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw",
            rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
            std::bind(&MonoNode::grabImage, this, std::placeholders::_1)
        );
        
        // TEMPORARILY COMMENT OUT THESE CALLS TO ISOLATE THE ISSUE
        // TODO: Uncomment these once the basic setup works
        /*
        it_ = std::make_shared<image_transport::ImageTransport>(
            std::static_pointer_cast<rclcpp::Node>(shared_from_this())
        );
        setup_publishers(this, *it_, this->get_name());
        setup_services(this, this->get_name());
        */
        
        std::cout << "Node setup complete" << std::endl;
    }
    
    ~MonoNode() override
    {
        if (pSLAM) {
            pSLAM->Shutdown();
            delete pSLAM;
            pSLAM = nullptr;
        }
    }

private:
    void grabImage(const sensor_msgs::msg::Image::ConstSharedPtr msg)
    {
        if (!pSLAM) {
            RCLCPP_WARN(this->get_logger(), "SLAM system not initialized");
            return;
        }
        
        // Convert to cv::Mat
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(msg);
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        
        // TrackMonocular expects seconds (double)
        const double tsec = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        (void)pSLAM->TrackMonocular(cv_ptr->image, tsec);
        
        // TEMPORARILY COMMENT OUT TO ISOLATE THE ISSUE
        // publish_topics(msg->header.stamp);
        
        static int frame_count = 0;
        if (++frame_count % 30 == 0) {  // Log every 30 frames
            RCLCPP_INFO(this->get_logger(), "Processed %d frames", frame_count);
        }
    }
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    std::string world_frame_id;
    std::string cam_frame_id;
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