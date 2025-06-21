// src/hik_camera_node.cpp

// 包含所有必要的头文件
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "rclcpp_components/register_node_macro.hpp" // <-- 用于注册组件
#include "image_transport/image_transport.hpp"

#include "hik_camera/hik_camera.hpp" // 您的海康相机驱动头文件

// 将所有代码包裹在一个命名空间中
namespace hik_camera
{

class HikCameraNode : public rclcpp::Node {
public:
    // 构造函数必须接受 NodeOptions
    explicit HikCameraNode(const rclcpp::NodeOptions & options) 
    : Node("hik_camera_node", options) 
    {
        RCLCPP_INFO(get_logger(), "Initializing Hikvision Camera Node (Standard Publisher)...");

        // 声明参数
        declare_parameter("frame_id", "camera");
        declare_parameter("frame_rate", 30.0);
        declare_parameter("exposure_time", 3000.0);
        declare_parameter("gain", 23.9);

        // 创建相机实例
        camera_ = std::make_unique<HikCamera>();

        // 初始化相机
        if (!camera_->openCamera()) {
            RCLCPP_ERROR(get_logger(), "Failed to initialize camera! Component will not be functional.");
            throw std::runtime_error("Failed to open camera");
        }

        // 设置相机参数
        camera_->setExposureTime(this->get_parameter("exposure_time").as_double());
        camera_->setGain(this->get_parameter("gain").as_double());

        // 使用 image_transport 发布，可自动支持压缩传输
        image_pub_ = image_transport::create_publisher(this, "camera/image");
        RCLCPP_INFO(get_logger(), "Publishing images on topic: %s", image_pub_.getTopic().c_str());

        // 创建定时器
        double frame_rate = this->get_parameter("frame_rate").as_double();
        auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / frame_rate));
        
        timer_ = create_wall_timer(
            timer_period,
            std::bind(&HikCameraNode::timer_callback, this));
    }

    ~HikCameraNode() {
        if (camera_) {
            camera_->closeCamera();
        }
    }

private:
    void timer_callback() {
        cv::Mat frame;
        if (camera_->getFrame(frame)) {
            // 创建消息头
            std_msgs::msg::Header header;
            header.stamp = this->now();
            header.frame_id = this->get_parameter("frame_id").as_string();

            // 使用 cv_bridge 转换图像
            auto msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
            
            // 通过 image_transport 发布（根据订阅者需求自动压缩）
            image_pub_.publish(*msg);
        } else {
            RCLCPP_WARN(get_logger(), "Failed to grab a frame from the camera.");
        }
    }

    std::unique_ptr<HikCamera> camera_;
    // image_transport 发布器
    image_transport::Publisher image_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace hik_camera

// 使用正确的命名空间和类名来注册组件
RCLCPP_COMPONENTS_REGISTER_NODE(hik_camera::HikCameraNode)