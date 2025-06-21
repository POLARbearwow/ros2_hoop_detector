#include <rclcpp/rclcpp.hpp>
#include "hik_camera/hik_camera.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<hik_camera::HikCameraNode>());
    rclcpp::shutdown();
    return 0;
} 