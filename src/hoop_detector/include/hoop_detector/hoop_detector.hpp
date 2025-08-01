#pragma once

#include <opencv2/opencv.hpp>
#include <chrono>
#include <map>
#include <string>
#include <vector>
#include <deque>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <image_transport/image_transport.hpp>

class HoopDetector {
public:
    HoopDetector(int kernel_size = 5, double solidity_threshold = 1.0, double min_score = 0.7);
    
    // 加载和处理图像
    HoopDetector& loadImage(const std::string& image_path);
    HoopDetector& loadImage(const cv::Mat& image);
    HoopDetector& createBinaryImage();
    HoopDetector& processImage();
    
    // 主要的检测函数
    std::pair<cv::Point, int> detectCircle();
    // 新增：椭圆检测函数
    cv::RotatedRect detectEllipse();
    // 使用椭圆特征解算PnP
    cv::Vec3f solvePnP(const cv::RotatedRect & ellipse);
    
    // 可视化和调试
    cv::Mat showProcess() const;
    void printTimingStats() const;
    
    // 获取处理后的图像
    const cv::Mat& getOriginalImage() const { return original_image_; }
    const cv::Mat& getInitialBinary() const { return initial_binary_; }
    const cv::Mat& getBinaryImage() const { return binary_image_; }
    const cv::Mat& getErodedImage() const { return eroded_image_; }
    const cv::Mat& getContourImage() const { return contour_image_; }
    const cv::Mat& getFilteredImage() const { return filtered_image_; }
    const cv::Mat& getFinalBinary() const { return final_binary_; }
    const cv::Mat& getResultImage() const { return result_image_; }
    const cv::Mat& getLargestContourImage() const { return largest_contour_image_; }
    const cv::Mat& getRemainingContoursImage() const { return remaining_contours_image_; }
    const cv::Mat& getSquareRegionImage() const { return square_region_image_; }

    void resetTiming();

    // 新增PnP相关函数
    void initializeCameraParams();
    cv::Vec3f solvePnP(const cv::Point& center, int radius);
    void setCameraParams(const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs);

    // 位置平滑
    cv::Vec3f getSmoothedPosition(const cv::Vec3f & new_pos);
    // double getSmoothedDistance(double new_distance);  // 添加这行

private:
    // 新增：日志控制相关
    std::chrono::steady_clock::time_point last_log_time_;
    const double log_interval_ = 1.0;  // 日志输出间隔（秒）
    void logIfNeeded(const std::string& message);
    
    // 新增：圆拟合相关
    std::pair<cv::Point2f, float> fitCircle3Points(
        const cv::Point2f& p1, const cv::Point2f& p2, const cv::Point2f& p3);
    std::pair<cv::Point, int> fitCircleRANSAC(
        const std::vector<cv::Point>& points, 
        int n_iterations = 200, 
        float threshold_dist = 8.0);
    
    // 新增：椭圆拟合 RANSAC
    cv::RotatedRect fitEllipseRANSAC(
        const std::vector<cv::Point>& points,
        int n_iterations = 200,
        float threshold_norm = 0.1f);
    
    // 成员变量
    // CircleDetector circle_detector_;
    // HoopDetector circle_detector_;
    // std::unique_ptr<HoopDetector> circle_detector_; 
    int kernel_size_;
    double solidity_threshold_;
    cv::Mat kernel_;
    
    // 图像保存相关
    std::chrono::steady_clock::time_point last_save_time_;
    int save_counter_ = 0;
    const double save_interval_ = 2.0; // 保存间隔（秒）
    
    // 图像处理过程中的中间结果
    cv::Mat original_image_;
    cv::Mat initial_binary_;
    cv::Mat binary_image_;
    cv::Mat eroded_image_;
    cv::Mat contour_image_;
    cv::Mat filtered_image_;
    cv::Mat largest_contour_image_;    // 最大轮廓图像
    cv::Mat remaining_contours_image_; // 剩余轮廓图像
    cv::Mat square_region_image_;     // 最像正方形的区域图像
    cv::Mat final_binary_;
    cv::Mat result_image_;
    
    // 时间统计
    std::map<std::string, double> timing_;

    // 相机参数
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    const double HOOP_DIAMETER_METERS = 0.45;  // 篮筐直径（米）

    // 平滑相关
    static constexpr int N_FRAMES_SMOOTHING = 15; // 滑动窗口大小
    static constexpr int N_TRIM = 2;             // 截尾个数
    std::deque<cv::Vec3f> recent_positions_;
    std::deque<double>    recent_distances_;
    
    // 辅助函数
    void getRPY(const cv::Mat& R, double& roll, double& pitch, double& yaw);
}; 

// =====================================
// ROS 2 Component node for HoopDetector
// =====================================
namespace hoop_detector {

class HoopDetectorNode : public rclcpp::Node {
public:
    explicit HoopDetectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

    ::HoopDetector detector_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher  debug_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_smooth_pub_;

    std::string camera_frame_;
};

} // namespace hoop_detector 