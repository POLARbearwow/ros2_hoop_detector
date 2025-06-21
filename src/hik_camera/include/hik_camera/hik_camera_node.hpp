#pragma once

#include "hik_camera/hik_camera.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include "rclcpp_components/register_node_macro.hpp"

namespace hik_camera {

class HikCameraNode : public rclcpp::Node {
public:
    explicit HikCameraNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    virtual ~HikCameraNode();

private:
    void timerCallback();
    void declareParameters();
    void updateCameraParams();

    // 相机实例
    HikCamera camera_;
    
    // ROS2 组件
    rclcpp::TimerBase::SharedPtr timer_;
    image_transport::Publisher image_pub_;
    
    // 参数
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    float exposure_time_;
    float gain_;
    int fps_;
};

} // namespace hik_camera 

// RCLCPP_COMPONENTS_REGISTER_NODE(my_namespace::hik_camera_component)