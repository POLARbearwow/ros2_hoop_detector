#include <rclcpp/rclcpp.hpp>
#include "hoop_detector/hoop_detector.hpp"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "rclcpp_components/register_node_macro.hpp"

using std::placeholders::_1;

namespace hoop_detector {

HoopDetectorNode::HoopDetectorNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("hoop_detector_node", options)
{
    // 参数
    std::string input_topic = this->declare_parameter<std::string>("input_image_topic", "camera/image");
    std::string debug_topic = this->declare_parameter<std::string>("debug_image_topic", "hoop_detector/debug_image");
    std::string pose_topic  = this->declare_parameter<std::string>("pose_topic", "hoop_detector/pose");
    camera_frame_          = this->declare_parameter<std::string>("camera_frame", "camera");

    // 订阅图像
    image_sub_ = image_transport::create_subscription(
        this,
        input_topic,
        std::bind(&HoopDetectorNode::imageCallback, this, _1),
        "raw");

    RCLCPP_INFO(get_logger(), "Subscribed to image topic: %s", input_topic.c_str());

    // 发布调试图像
    debug_pub_ = image_transport::create_publisher(this, debug_topic);
    RCLCPP_INFO(get_logger(), "Publishing debug images on: %s", debug_topic.c_str());

    // 发布位姿
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic, 10);
    RCLCPP_INFO(get_logger(), "Publishing pose on: %s", pose_topic.c_str());
}

void HoopDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
    // 将 ROS 图像转换为 OpenCV
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception & e) {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    auto & frame = cv_ptr->image;

    // 调用算法
    try {
        detector_.resetTiming();
        detector_.loadImage(frame)
                 .createBinaryImage()
                 .processImage();
        auto [center, radius] = detector_.detectCircle();
        // const cv::Point & center = circle.first;
        // int radius = circle.second;

        if (radius > 0) {
            cv::Vec3f tvec = detector_.solvePnP(center, radius);

            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header = msg->header;
            pose_msg.header.frame_id = camera_frame_;
            pose_msg.pose.position.x = tvec[0];
            pose_msg.pose.position.y = tvec[1];
            pose_msg.pose.position.z = tvec[2];
            pose_msg.pose.orientation.w = 1.0; // 暂无姿态
            pose_pub_->publish(pose_msg);
        }

        // 发布调试图像
        const cv::Mat & debug_image = detector_.getResultImage();
        if (!debug_image.empty() && debug_pub_.getNumSubscribers() > 0) {
            std_msgs::msg::Header header = msg->header;
            auto debug_msg = cv_bridge::CvImage(header, "bgr8", debug_image).toImageMsg();
            debug_pub_.publish(*debug_msg);
        }
    } catch (const std::exception & e) {
        RCLCPP_WARN(get_logger(), "Hoop detection failed: %s", e.what());
    }
}

} // namespace hoop_detector

RCLCPP_COMPONENTS_REGISTER_NODE(hoop_detector::HoopDetectorNode) 