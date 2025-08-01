#include "hoop_detector/hoop_detector.hpp"
#include <iostream>
#include <random>
#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <set>

void HoopDetector::logIfNeeded(const std::string& message) {
    auto current_time = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(current_time - last_log_time_).count();
    if (elapsed >= log_interval_) {
        std::cout << message << std::endl;
        last_log_time_ = current_time;
    }
}

HoopDetector::HoopDetector(int kernel_size, double solidity_threshold, double min_score)
    :kernel_size_(kernel_size)
    , solidity_threshold_(solidity_threshold)
    , last_save_time_(std::chrono::steady_clock::now())
    , last_log_time_(std::chrono::steady_clock::now()) {
    std::cout << "[HoopDetector] 初始化参数: kernel_size=" << kernel_size 
              << ", solidity_threshold=" << solidity_threshold 
              << ", min_score=" << min_score << std::endl;
    kernel_ = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernel_size_, kernel_size_));
}

void HoopDetector::resetTiming() {
    timing_.clear();
}

HoopDetector& HoopDetector::loadImage(const std::string& image_path) {
    auto start = std::chrono::high_resolution_clock::now();
    
    original_image_ = cv::imread(image_path);
    if (original_image_.empty()) {
        throw std::runtime_error("Failed to load image from " + image_path);
    }
    // circle_detector_.loadImage(original_image_);
    
    auto end = std::chrono::high_resolution_clock::now();
    timing_["load_image"] = std::chrono::duration<double>(end - start).count();
    
    return *this;
}

HoopDetector& HoopDetector::loadImage(const cv::Mat& image) {
    auto start = std::chrono::high_resolution_clock::now();
    
    if (image.empty()) {
        throw std::runtime_error("[HoopDetector] 输入图像为空");
    }
    
    // 确保图像是BGR格式
    if (image.type() != CV_8UC3) {
        if (image.channels() == 3) {
            cv::Mat temp;
            image.convertTo(temp, CV_8UC3);
            original_image_ = temp.clone();
        } else {
            throw std::runtime_error("[HoopDetector] 不支持的图像格式");
        }
    } else {
        original_image_ = image.clone();
    }
    
    // circle_detector_.loadImage(original_image_);
    
    auto end = std::chrono::high_resolution_clock::now();
    timing_["load_image"] = std::chrono::duration<double>(end - start).count();
    
    return *this;
}

HoopDetector& HoopDetector::createBinaryImage() {
    if (original_image_.empty()) {
        throw std::runtime_error("[HoopDetector] 未加载图像");
    }
    
    auto start = std::chrono::high_resolution_clock::now();
    
    try {
        // logIfNeeded("[HoopDetector] 开始图像处理...");
        
        // 转换到HSV颜色空间
        cv::Mat hsv_frame;
        cv::cvtColor(original_image_, hsv_frame, cv::COLOR_BGR2HSV);
        
        // 创建橙色掩码
        cv::Scalar lower_bound(0, 125, 72);
        cv::Scalar upper_bound(13, 255, 153);
        cv::inRange(hsv_frame, lower_bound, upper_bound, initial_binary_);
        
        if (initial_binary_.empty()) {
            throw std::runtime_error("[HoopDetector] 二值图创建失败");
        }
        
        // 清理掩码
        // cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        // cv::morphologyEx(initial_binary_, initial_binary_, cv::MORPH_CLOSE, element);
        cv::medianBlur(initial_binary_, binary_image_, 23);
        
    } catch (const cv::Exception& e) {
        std::cerr << "[HoopDetector] OpenCV错误: " << e.what() << std::endl;
        throw;
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    timing_["create_binary"] = std::chrono::duration<double>(end - start).count();
    
    return *this;
}

HoopDetector& HoopDetector::processImage() {
    if (binary_image_.empty()) {
        throw std::runtime_error("[HoopDetector] Binary image not created");
    }
    
    auto start = std::chrono::high_resolution_clock::now();
    
    try {
        // std::cout << "[HoopDetector] 开始处理图像..." << std::endl;
        
        // 1. 直接使用binary_image_作为最终的二值图
        final_binary_ = binary_image_.clone();
        
        // 2. 保存中间结果供显示使用
        filtered_image_ = binary_image_.clone();  // 保存滤波后的二值图
        
        // std::cout << "[HoopDetector] 图像处理完成" << std::endl;
        
    } catch (const cv::Exception& e) {
        std::cerr << "[HoopDetector] OpenCV错误: " << e.what() << std::endl;
        throw;
    } catch (const std::exception& e) {
        std::cerr << "[HoopDetector] 标准错误: " << e.what() << std::endl;
        throw;
    } catch (...) {
        std::cerr << "[HoopDetector] 未知错误!" << std::endl;
        throw;
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    timing_["process_image"] = std::chrono::duration<double>(end - start).count();
    
    return *this;
}

std::pair<cv::Point, int> HoopDetector::detectCircle() {
    if (final_binary_.empty()) {
        throw std::runtime_error("[HoopDetector] 图像未处理");
    }
    
    auto start = std::chrono::high_resolution_clock::now();
    
    try {
        // logIfNeeded("[HoopDetector] 开始圆检测...");
        
        // 提取所有轮廓点到一个向量中
        std::vector<cv::Point> all_contour_points;
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(final_binary_, contours, hierarchy, 
                        cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        
        // 将所有轮廓的点合并到一个向量中
        for(const auto& contour : contours) {
            all_contour_points.insert(all_contour_points.end(), contour.begin(), contour.end());
        }
        
        logIfNeeded("[HoopDetector] 总共提取了 " + std::to_string(all_contour_points.size()) + " 个轮廓点");
        
        // 在原始图像和二值图上绘制结果
        result_image_ = original_image_.clone();
        cv::Mat binary_result;
        cv::cvtColor(final_binary_, binary_result, cv::COLOR_GRAY2BGR);

        auto result = fitCircleRANSAC(all_contour_points);
        auto center = result.first;
        auto radius = result.second;
        
        if (radius > 0) {
            logIfNeeded(
                "[HoopDetector] 检测结果: 中心=(" + std::to_string(center.x) + "," + 
                std::to_string(center.y) + ") 半径=" + std::to_string(radius)
            );
            
            // 计算四个特征点（上下左右）
            cv::Point top(center.x, center.y - radius);
            cv::Point bottom(center.x, center.y + radius);
            cv::Point left(center.x - radius, center.y);
            cv::Point right(center.x + radius, center.y);
            
            // 在原始图像上绘制
            // 1. 绘制圆和圆心
            cv::circle(result_image_, center, radius, cv::Scalar(0, 255, 0), 4);
            cv::circle(result_image_, center, 8, cv::Scalar(0, 0, 255), -1);
            
            // 2. 绘制四个特征点（使用不同颜色）
            cv::circle(result_image_, top, 6, cv::Scalar(255, 0, 0), -1);
            cv::circle(result_image_, bottom, 6, cv::Scalar(255, 0, 0), -1);
            cv::circle(result_image_, left, 6, cv::Scalar(255, 0, 0), -1);
            cv::circle(result_image_, right, 6, cv::Scalar(255, 0, 0), -1);
            
            // 3. 添加坐标标注
            std::string center_text = "C(" + std::to_string(center.x) + "," + std::to_string(center.y) + ")";
            std::string top_text = "T(" + std::to_string(top.x) + "," + std::to_string(top.y) + ")";
            std::string bottom_text = "B(" + std::to_string(bottom.x) + "," + std::to_string(bottom.y) + ")";
            std::string left_text = "L(" + std::to_string(left.x) + "," + std::to_string(left.y) + ")";
            std::string right_text = "R(" + std::to_string(right.x) + "," + std::to_string(right.y) + ")";
            
            // 设置文本参数
            int font = cv::FONT_HERSHEY_SIMPLEX;
            double font_scale = 0.6;
            int thickness = 2;
            cv::Scalar text_color(255, 255, 255);
            
            // 绘制文本（带黑色边框以提高可读性）
            auto drawText = [&](const std::string& text, const cv::Point& point) {
                cv::putText(result_image_, text, 
                           cv::Point(point.x + 10, point.y + 10), 
                           font, font_scale, cv::Scalar(0, 0, 0), thickness + 1);
                cv::putText(result_image_, text, 
                           cv::Point(point.x + 10, point.y + 10), 
                           font, font_scale, text_color, thickness);
            };
            
            drawText(center_text, center);
            drawText(top_text, top);
            drawText(bottom_text, bottom);
            drawText(left_text, left);
            drawText(right_text, right);
            
            // 在二值图上也绘制相同的标注
            cv::circle(binary_result, center, radius, cv::Scalar(0, 255, 0), 4);
            cv::circle(binary_result, center, 8, cv::Scalar(0, 0, 255), -1);
            cv::circle(binary_result, top, 6, cv::Scalar(255, 0, 0), -1);
            cv::circle(binary_result, bottom, 6, cv::Scalar(255, 0, 0), -1);
            cv::circle(binary_result, left, 6, cv::Scalar(255, 0, 0), -1);
            cv::circle(binary_result, right, 6, cv::Scalar(255, 0, 0), -1);
            
            // 更新final_binary_为带有拟合结果的图像
            cv::cvtColor(binary_result, final_binary_, cv::COLOR_BGR2GRAY);
            
            // 执行PnP解算
            cv::Vec3f position = solvePnP(center, radius);
            
            auto end = std::chrono::high_resolution_clock::now();
            timing_["detect_circle"] = std::chrono::duration<double>(end - start).count();
            
            // 计算并输出性能统计
            double total_time = 0;
            for (const auto& [_, duration] : timing_) {
                total_time += duration;
            }
            
            logIfNeeded(
                "[HoopDetector] 性能统计: 总耗时=" + std::to_string(total_time * 1000) + 
                "ms, FPS=" + std::to_string(1.0 / total_time)
            );
            
            return std::make_pair(center, radius);
        }
        
        logIfNeeded("[HoopDetector] 未检测到圆");
        return std::make_pair(cv::Point(0, 0), 0);
        
    } catch (const cv::Exception& e) {
        std::cerr << "[HoopDetector] OpenCV错误: " << e.what() << std::endl;
        throw;
    } catch (const std::exception& e) {
        std::cerr << "[HoopDetector] 标准错误: " << e.what() << std::endl;
        throw;
    } catch (...) {
        std::cerr << "[HoopDetector] 未知错误!" << std::endl;
        throw;
    }
}

void HoopDetector::printTimingStats() const {
    double total_time = 0;
    for (const auto& [_, duration] : timing_) {
        total_time += duration;
    }
    
    std::cout << "\n=== 性能统计 ===" << std::endl;
    for (const auto& [step, duration] : timing_) {
        double percentage = (duration / total_time) * 100;
        printf("%-15s: %6.2f ms (%5.1f%%)\n", 
               step.c_str(), duration * 1000, percentage);
    }
    
    std::cout << std::string(40, '-') << std::endl;
    printf("%-15s: %6.2f ms\n", "总耗时", total_time * 1000);
    printf("FPS          : %6.2f\n", 1.0 / total_time);
}

cv::Mat HoopDetector::showProcess() const {
    if (original_image_.empty()) {
        throw std::runtime_error("No image loaded");
    }
    
    // 准备所有处理步骤的图像
    std::vector<cv::Mat> images;
    std::vector<std::string> titles;
    
    // 添加原始图像
    images.push_back(original_image_);
    titles.push_back("1. Original Image");
    
    // 添加初始二值图像
    if (!initial_binary_.empty()) {
        cv::Mat initial_binary_display;
        cv::cvtColor(initial_binary_, initial_binary_display, cv::COLOR_GRAY2BGR);
        images.push_back(initial_binary_display);
        titles.push_back("2. Initial Binary");
    }
    
    // 添加滤波后的二值图像
    if (!binary_image_.empty()) {
        cv::Mat binary_display;
        cv::cvtColor(binary_image_, binary_display, cv::COLOR_GRAY2BGR);
        images.push_back(binary_display);
        titles.push_back("3. Filtered Binary");
    }
    
    // 添加最终的二值图像
    if (!final_binary_.empty()) {
        cv::Mat final_display;
        cv::cvtColor(final_binary_, final_display, cv::COLOR_GRAY2BGR);
        images.push_back(final_display);
        titles.push_back("4. Final Binary");
    }
    
    // 添加结果图像
    if (!result_image_.empty()) {
        images.push_back(result_image_);
        titles.push_back("5. Detection Result");
    }
    
    // 计算显示布局
    int n_images = images.size();
    int n_cols = std::min(3, n_images);  // 每行最多显示3张图
    int n_rows = (n_images + n_cols - 1) / n_cols;
    
    // 调整图像大小
    const int height = 300;
    std::vector<cv::Mat> processed_images;
    int max_width = 0;
    
    for (const auto& img : images) {
        double aspect_ratio = static_cast<double>(img.cols) / img.rows;
        int width = static_cast<int>(height * aspect_ratio);
        cv::Mat resized;
        cv::resize(img, resized, cv::Size(width, height));
        processed_images.push_back(resized);
        max_width = std::max(max_width, width);
    }
    
    // 创建画布
    cv::Mat canvas = cv::Mat::zeros(height * n_rows, max_width * n_cols, CV_8UC3);
    
    // 放置图像和标题
    for (int idx = 0; idx < n_images; ++idx) {
        int i = idx / n_cols;
        int j = idx % n_cols;
        int y_start = i * height;
        int x_start = j * max_width + (max_width - processed_images[idx].cols) / 2;
        
        // 放置图像
        processed_images[idx].copyTo(
            canvas(cv::Rect(x_start, y_start, 
                          processed_images[idx].cols, 
                          processed_images[idx].rows)));
        
        // 添加标题
        cv::putText(canvas, titles[idx], 
                   cv::Point(x_start + 10, y_start + 30),
                   cv::FONT_HERSHEY_SIMPLEX, 0.8, 
                   cv::Scalar(0, 255, 0), 2);
    }
    
    return canvas;
}

std::pair<cv::Point, int> HoopDetector::fitCircleRANSAC(
    const std::vector<cv::Point>& points, 
    int n_iterations, 
    float threshold_dist) {
    
    if (points.size() < 3) {
        return std::make_pair(cv::Point(0, 0), 0);
    }

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, points.size() - 1);

    cv::Point best_center(0, 0);
    int best_radius = 0;
    int max_inliers = 0;

    for (int iter = 0; iter < n_iterations; ++iter) {
        // 随机选择3个点
        int idx1 = dis(gen);
        int idx2 = dis(gen);
        int idx3 = dis(gen);

        // 确保选择的点不重复
        if (idx1 == idx2 || idx2 == idx3 || idx1 == idx3) {
            continue;
        }

        cv::Point2f p1 = points[idx1];
        cv::Point2f p2 = points[idx2];
        cv::Point2f p3 = points[idx3];

        // 计算三点确定的圆
        float x1 = p1.x, y1 = p1.y;
        float x2 = p2.x, y2 = p2.y;
        float x3 = p3.x, y3 = p3.y;

        float a = x1 * (y2 - y3) - y1 * (x2 - x3) + x2 * y3 - x3 * y2;
        float b = (x1 * x1 + y1 * y1) * (y3 - y2) + 
                 (x2 * x2 + y2 * y2) * (y1 - y3) + 
                 (x3 * x3 + y3 * y3) * (y2 - y1);
        float c = (x1 * x1 + y1 * y1) * (x2 - x3) + 
                 (x2 * x2 + y2 * y2) * (x3 - x1) + 
                 (x3 * x3 + y3 * y3) * (x1 - x2);

        // 检查是否有效的圆
        if (std::abs(a) < 1e-7) {
            continue;
        }

        float center_x = -b / (2 * a);
        float center_y = -c / (2 * a);
        cv::Point2f center(center_x, center_y);

        // 计算半径
        float radius = std::sqrt(std::pow(x1 - center_x, 2) + std::pow(y1 - center_y, 2));

        // 计算内点数量
        int inliers = 0;
        for (const auto& point : points) {
            float dist = std::abs(std::sqrt(std::pow(point.x - center_x, 2) + 
                                          std::pow(point.y - center_y, 2)) - radius);
            if (dist < threshold_dist) {
                inliers++;
            }
        }

        // 更新最佳结果
        if (inliers > max_inliers && inliers > 150) {
            max_inliers = inliers;
            best_center = cv::Point(std::round(center_x), std::round(center_y));
            best_radius = std::round(radius);
        }
    }

    return std::make_pair(best_center, best_radius);
}

void HoopDetector::initializeCameraParams() {
    camera_matrix_ = (cv::Mat_<double>(3, 3) << 
        1.8047987054698410e+03, 0., 7.0794773098538269e+02, 
        0., 1.8040138677418131e+03, 5.4521886784352614e+02, 
        0., 0., 1. );
    
    // 假设没有畸变
    dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);
    
    std::cout << "[HoopDetector] 相机参数初始化完成" << std::endl;
    std::cout << "相机内参矩阵:\n" << camera_matrix_ << std::endl;
}

void HoopDetector::setCameraParams(const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs) {
    camera_matrix_ = camera_matrix.clone();
    dist_coeffs_ = dist_coeffs.clone();
}

void HoopDetector::getRPY(const cv::Mat& R, double& roll, double& pitch, double& yaw) {
    // 从旋转矩阵计算欧拉角
    roll = atan2(R.at<double>(2,1), R.at<double>(2,2));
    pitch = atan2(-R.at<double>(2,0), 
                  sqrt(R.at<double>(2,1)*R.at<double>(2,1) + R.at<double>(2,2)*R.at<double>(2,2)));
    yaw = atan2(R.at<double>(1,0), R.at<double>(0,0));
    
    // 转换为角度
    roll = roll * 180.0 / CV_PI;
    pitch = pitch * 180.0 / CV_PI;
    yaw = yaw * 180.0 / CV_PI;
}

cv::Vec3f HoopDetector::solvePnP(const cv::Point& center, int radius) {
    if (camera_matrix_.empty()) {
        initializeCameraParams();
    }
    
    try {
        // 构建3D空间点（假设篮筐在世界坐标系原点，平面平行于XY平面）
        std::vector<cv::Point3f> object_points;
        const float RADIUS = HOOP_DIAMETER_METERS / 2.0;
        object_points.push_back(cv::Point3f(0, RADIUS, 0));   // 上
        object_points.push_back(cv::Point3f(0, -RADIUS, 0));  // 下
        object_points.push_back(cv::Point3f(-RADIUS, 0, 0));  // 左
        object_points.push_back(cv::Point3f(RADIUS, 0, 0));   // 右

        // 构建对应的图像点
        std::vector<cv::Point2f> image_points;
        image_points.push_back(cv::Point2f(center.x, center.y - radius)); // 上
        image_points.push_back(cv::Point2f(center.x, center.y + radius)); // 下
        image_points.push_back(cv::Point2f(center.x - radius, center.y)); // 左
        image_points.push_back(cv::Point2f(center.x + radius, center.y)); // 右

        // 求解PnP
        cv::Mat rvec, tvec;
        bool success = cv::solvePnP(object_points, image_points, 
                                  camera_matrix_, dist_coeffs_, 
                                  rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);

        if (success) {
            // 将旋转向量转换为旋转矩阵
            cv::Mat R;
            cv::Rodrigues(rvec, R);
            
            // 计算欧拉角
            double roll, pitch, yaw;
            getRPY(R, roll, pitch, yaw);
            
            // 计算距离
            double distance = cv::norm(tvec);
            
            // 输出结果
            std::cout << "\n=== PnP解算结果 ===" << std::endl;
            std::cout << "位置 (meters):" << std::endl;
            std::cout << "X: " << tvec.at<double>(0) << std::endl;
            std::cout << "Y: " << tvec.at<double>(1) << std::endl;
            std::cout << "Z: " << tvec.at<double>(2) << std::endl;
            std::cout << "距离: " << distance << " meters" << std::endl;
            std::cout << "\n姿态 (degrees):" << std::endl;
            std::cout << "Roll : " << roll << std::endl;
            std::cout << "Pitch: " << pitch << std::endl;
            std::cout << "Yaw  : " << yaw << std::endl;
            std::cout << "===================" << std::endl;
            
            return cv::Vec3f(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
        } else {
            std::cerr << "[HoopDetector] PnP求解失败" << std::endl;
            return cv::Vec3f(0, 0, 0);
        }
        
    } catch (const cv::Exception& e) {
        std::cerr << "[HoopDetector] OpenCV错误: " << e.what() << std::endl;
        return cv::Vec3f(0, 0, 0);
    } catch (const std::exception& e) {
        std::cerr << "[HoopDetector] 标准错误: " << e.what() << std::endl;
        return cv::Vec3f(0, 0, 0);
    }
}

// =============================================
// 椭圆拟合 RANSAC 辅助函数
// =============================================
cv::RotatedRect HoopDetector::fitEllipseRANSAC(
    const std::vector<cv::Point>& points,
    int n_iterations,
    float threshold_norm) {

    // 至少 5 个点才能拟合椭圆
    if (points.size() < 5) {
        return cv::RotatedRect();
    }

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, static_cast<int>(points.size() - 1));

    int best_inliers = 0;
    cv::RotatedRect best_ellipse;

    auto countInliers = [&](const cv::RotatedRect & ellipse) {
        if (ellipse.size.width <= 0 || ellipse.size.height <= 0) return 0;

        double a = ellipse.size.width  / 2.0;  // 半长轴
        double b = ellipse.size.height / 2.0;  // 半短轴
        double angle_rad = ellipse.angle * CV_PI / 180.0;
        double cosA = std::cos(angle_rad);
        double sinA = std::sin(angle_rad);

        int inliers = 0;
        for (const auto & pt : points) {
            double dx = pt.x - ellipse.center.x;
            double dy = pt.y - ellipse.center.y;

            // 坐标旋转到椭圆轴对齐坐标系
            double xRot =  dx * cosA + dy * sinA;
            double yRot = -dx * sinA + dy * cosA;

            double norm = (xRot * xRot) / (a * a) + (yRot * yRot) / (b * b);
            if (std::fabs(norm - 1.0) < threshold_norm) {
                ++inliers;
            }
        }
        return inliers;
    };

    for (int iter = 0; iter < n_iterations; ++iter) {
        // 随机采样 5 个不重复的点
        std::set<int> idx_set;
        while (idx_set.size() < 5) {
            idx_set.insert(dis(gen));
        }

        std::vector<cv::Point> sample;
        sample.reserve(5);
        for (int idx : idx_set) {
            sample.push_back(points[idx]);
        }

        cv::RotatedRect candidate;
        try {
            candidate = cv::fitEllipse(sample);
        } catch (const cv::Exception &) {
            continue; // 拟合失败，继续下一轮
        }

        int inliers = countInliers(candidate);
        if (inliers > best_inliers) {
            best_inliers = inliers;
            best_ellipse = candidate;
        }
    }

    if (best_inliers == 0) {
        return cv::RotatedRect();
    }

    // 重新收集内点并做一次精细拟合
    std::vector<cv::Point> inlier_pts;
    double a = best_ellipse.size.width  / 2.0;
    double b = best_ellipse.size.height / 2.0;
    double angle_rad = best_ellipse.angle * CV_PI / 180.0;
    double cosA = std::cos(angle_rad);
    double sinA = std::sin(angle_rad);

    for (const auto & pt : points) {
        double dx = pt.x - best_ellipse.center.x;
        double dy = pt.y - best_ellipse.center.y;
        double xRot =  dx * cosA + dy * sinA;
        double yRot = -dx * sinA + dy * cosA;
        double norm = (xRot * xRot) / (a * a) + (yRot * yRot) / (b * b);
        if (std::fabs(norm - 1.0) < threshold_norm) {
            inlier_pts.push_back(pt);
        }
    }

    if (inlier_pts.size() >= 5) {
        try {
            best_ellipse = cv::fitEllipse(inlier_pts);
        } catch (const cv::Exception &) {
            // 保留原先最佳
        }
    }

    return best_ellipse;
}

// =============================================
// 椭圆检测主函数
// =============================================
cv::RotatedRect HoopDetector::detectEllipse() {
    if (final_binary_.empty()) {
        throw std::runtime_error("[HoopDetector] 图像未处理");
    }

    // 提取所有轮廓点
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(final_binary_, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<cv::Point> all_points;
    for (const auto & c : contours) {
        all_points.insert(all_points.end(), c.begin(), c.end());
    }

    if (all_points.size() < 5) {
        logIfNeeded("[HoopDetector] 点数量不足，无法拟合椭圆");
        return cv::RotatedRect();
    }

    cv::RotatedRect ellipse = fitEllipseRANSAC(all_points);

    // 将 final_binary_ 转为可视化图像
    cv::cvtColor(final_binary_, result_image_, cv::COLOR_GRAY2BGR);

    if (ellipse.size.width > 0 && ellipse.size.height > 0) {
        cv::ellipse(result_image_, ellipse, cv::Scalar(0,255,0), 3);
        // 绘制所有轮廓线，便于调试
        for(const auto & c : contours){
            std::vector<std::vector<cv::Point>> cs = {c};
            cv::drawContours(result_image_, cs, -1, cv::Scalar(0, 150, 255), 1);
        }
        cv::circle(result_image_, ellipse.center, 5, cv::Scalar(0,0,255), -1);

        // Solve PnP with ellipse if needed
        // solvePnP(ellipse);
    }

    return ellipse;
}

// =============================================
// 椭圆解算 PnP
// =============================================
cv::Vec3f HoopDetector::solvePnP(const cv::RotatedRect& ellipse) {
    if (ellipse.size.width <= 0 || ellipse.size.height <= 0) {
        return cv::Vec3f(0,0,0);
    }

    if (camera_matrix_.empty()) {
        initializeCameraParams();
    }

    // 取 6 个特征点：主轴两端、次轴两端、45° 两点
    float a = ellipse.size.width  / 2.0f;   // 半长轴像素
    float b = ellipse.size.height / 2.0f;   // 半短轴像素
    float angle = ellipse.angle * CV_PI / 180.0f;

    auto rot = [&](float x, float y){
        float xr = x * std::cos(angle) - y * std::sin(angle);
        float yr = x * std::sin(angle) + y * std::cos(angle);
        return cv::Point2f(ellipse.center.x + xr, ellipse.center.y + yr);
    };

    std::vector<cv::Point2f> image_points = {
        rot( a, 0),    // +X
        rot(-a, 0),    // -X
        rot(0,  b),    // +Y
        rot(0, -b),    // -Y
        rot( a*std::sqrt(0.5f),  b*std::sqrt(0.5f)),  // 45°
        rot(-a*std::sqrt(0.5f),  b*std::sqrt(0.5f))   // 135°
    };

    // 对应 3D 点（假设篮筐圆环在 Z=0 平面，直径已知）
    float A = HOOP_DIAMETER_METERS / 2.0f; // 长轴半径
    float B = A * (b / a);                 // 根据像素轴比估计短轴半径 (近似)

    std::vector<cv::Point3f> object_points = {
        { A, 0, 0}, {-A, 0, 0}, {0, B, 0}, {0,-B,0},
        { A*0.7071f,  B*0.7071f, 0}, {-A*0.7071f,  B*0.7071f, 0}
    };

    cv::Mat rvec, tvec;
    bool success = cv::solvePnP(object_points, image_points,
                                camera_matrix_, dist_coeffs_, rvec, tvec, false,
                                cv::SOLVEPNP_ITERATIVE);
    if (!success) {
        return cv::Vec3f(0,0,0);
    }
    return cv::Vec3f(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
}

// double HoopDetector::getSmoothedDistance(double new_distance) {
//     // 1. 更新滑动窗口
//     recent_distances_.push_back(new_distance);
//     if (recent_distances_.size() > N_FRAMES_SMOOTHING) {
//         recent_distances_.pop_front(); // 移除最旧的数据，维持窗口大小
//     }

//     // 2. 检查窗口内的数据是否足够进行截尾平均
//     // 至少需要 (2 * N_TRIM + 1) 个数据点才能执行截尾
//     if (recent_distances_.size() < (2 * N_TRIM + 1)) {
//         // 如果数据不足，则返回简单的移动平均值
//         if (recent_distances_.empty()) {
//             return 0.0; // 或者返回一个表示无效的值
//         }
//         double sum = std::accumulate(recent_distances_.begin(), recent_distances_.end(), 0.0);
//         return sum / recent_distances_.size();
//     }

//     // 3. 执行截尾平均
//     // a. 将当前窗口的数据复制到一个vector中，以便排序
//     std::vector<double> sorted_window(recent_distances_.begin(), recent_distances_.end());

//     // b. 对复制的窗口进行排序
//     std::sort(sorted_window.begin(), sorted_window.end());

//     // c. 计算剔除首尾元素后的和
//     // 使用迭代器范围来求和，跳过开头N_TRIM个和结尾N_TRIM个元素
//     double sum = std::accumulate(
//         sorted_window.begin() + N_TRIM, // 跳过N_TRIM个最小值
//         sorted_window.end() - N_TRIM,   // 跳过N_TRIM个最大值
//         0.0
//     );

//     // d. 计算平均值
//     int count_to_average = sorted_window.size() - (2 * N_TRIM);
//     return sum / count_to_average;
// }

cv::Vec3f HoopDetector::getSmoothedPosition(const cv::Vec3f & new_pos) {
    // 更新窗口
    recent_positions_.push_back(new_pos);
    if (recent_positions_.size() > N_FRAMES_SMOOTHING) {
        recent_positions_.pop_front();
    }

    size_t N = recent_positions_.size();
    if (N < static_cast<size_t>(2 * N_TRIM + 1)) {
        // 简单平均
        cv::Vec3f sum(0,0,0);
        for (const auto & p : recent_positions_) sum += p;
        return sum * (1.0f / static_cast<float>(N));
    }

    // 拆分三个维度
    std::vector<float> xs, ys, zs;
    xs.reserve(N); ys.reserve(N); zs.reserve(N);
    for (const auto & p : recent_positions_) {
        xs.push_back(p[0]); ys.push_back(p[1]); zs.push_back(p[2]);
    }
    auto trimmedMean = [&](std::vector<float> & vec) -> float {
        std::sort(vec.begin(), vec.end());
        float sum = 0.0f;
        for (size_t i = N_TRIM; i < vec.size() - N_TRIM; ++i) sum += vec[i];
        return sum / static_cast<float>(vec.size() - 2 * N_TRIM);
    };

    return cv::Vec3f(trimmedMean(xs), trimmedMean(ys), trimmedMean(zs));
}