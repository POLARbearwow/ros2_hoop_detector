#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include "hik_camera.hpp"
#include "hoop_detector.hpp"
#include "camera_calibrator.hpp"
#include "image_saver.hpp"
#include "MvCameraControl.h"
#include <unistd.h>

void printUsage() {
    std::cout << "使用方法:\n"
              << "1. 摄像头模式: ./hoop_detecor --camera\n"
              << "2. 图片模式: ./hoop_detecor --image <图片路径>\n"
              << "3. 标定模式: ./hoop_detecor --calibrate <棋盘格宽> <棋盘格高> <方格大小(mm)> <图片目录>\n"
              << "4. 验证标定: ./hoop_detecor --verify <标定文件> <测试图片>\n"
              << "5. 保存图片: ./hoop_detecor --save [保存目录]\n"
              << "\n示例:\n"
              << "1. 摄像头模式: ./hoop_detecor --camera\n"
              << "2. 图片模式: ./hoop_detecor --image test.jpg\n"
              << "3. 标定模式: ./hoop_detecor --calibrate 9 6 20 ./chess_images/\n"
              << "4. 验证标定: ./hoop_detecor --verify camera_params.yml test.jpg\n"
              << "5. 保存图片: ./hoop_detecor --save ./captured_images" << std::endl;
}

// 处理单张图片的函数
bool processImage(const std::string& imagePath, HoopDetector& detector) {
    std::cout << "[Main] 正在处理图片: " << imagePath << std::endl;
    
    cv::Mat frame = cv::imread(imagePath);
    if (frame.empty()) {
        std::cerr << "[Main] 无法读取图片: " << imagePath << std::endl;
        return false;
    }

    try {
        // 处理图像
        detector.loadImage(frame)
               .createBinaryImage()
               .processImage();

        // 检测篮筐
        auto [center, radius] = detector.detectCircle();

        // 显示原始图像
        cv::imshow("Image", frame);

        // 显示处理步骤
        cv::Mat process_view = detector.showProcess();
        if (!process_view.empty()) {
            cv::imshow("Processing Steps", process_view);
        }

        std::cout << "[Main] 按任意键继续..." << std::endl;
        cv::waitKey(0);
        return true;
    }
    catch (const std::exception& e) {
        std::cerr << "[Main] 处理错误: " << e.what() << std::endl;
        return false;
    }
}

// 处理摄像头的函数
void processCameraStream(HikCamera& camera, HoopDetector& detector) {
    cv::Mat frame;
    char key = 0;
    bool running = true;
    int frame_count = 0;
    const int LOG_INTERVAL = 30; // 每30帧输出一次结果

    while (running && key != 'q') {
        if (!camera.getFrame(frame)) {
            // std::cerr << "[Main] 获取图像失败" << std::endl;
            continue;
        }

        // 分开调用loadImage和processImage
        detector.loadImage(frame);
        detector.createBinaryImage();
        detector.processImage();

        // 检测篮筐
        auto [center, radius] = detector.detectCircle();

        // 显示结果
        cv::imshow("Camera", frame);
        cv::imshow("Processing Steps", detector.showProcess());
        
        // 获取PnP结果并定期输出
        if (frame_count % LOG_INTERVAL == 0 && radius > 0) {
            // 计算位姿
            cv::Vec3f pose = detector.solvePnP(center, radius);
            
            // 输出位姿信息
            std::cout << "\n=== 篮筐位姿 (第 " << frame_count << " 帧) ===" << std::endl;
            std::cout << "位置 (米):" << std::endl;
            std::cout << "X: " << std::fixed << std::setprecision(3) << pose[0] << std::endl;
            std::cout << "Y: " << std::fixed << std::setprecision(3) << pose[1] << std::endl;
            std::cout << "Z: " << std::fixed << std::setprecision(3) << pose[2] << std::endl;
        }

        frame_count++;
        key = cv::waitKey(1);
    }
}

int main(int argc, char* argv[]) {
    try {
        std::cout << "[Main] 程序启动..." << std::endl;

        // 检查命令行参数
        if (argc < 2) {
            printUsage();
            return -1;
        }

        std::string mode(argv[1]);
        
        if (mode == "--camera") {
            // 摄像头模式
            HikCamera camera;
            if (!camera.openCamera()) {
                std::cerr << "[Main] 相机初始化失败" << std::endl;
                return -1;
            }
            std::cout << "[Main] 相机初始化成功！" << std::endl;

            // 创建篮筐检测器实例
            std::cout << "[Main] 创建篮筐检测器..." << std::endl;
            HoopDetector detector(5, 1, 0.7);

            // 尝试加载相机参数
            cv::FileStorage fs("camera_params.yml", cv::FileStorage::READ);
            if (fs.isOpened()) {
                cv::Mat camera_matrix, dist_coeffs;
                fs["camera_matrix"] >> camera_matrix;
                fs["distortion_coefficients"] >> dist_coeffs;
                detector.setCameraParams(camera_matrix, dist_coeffs);
                std::cout << "[Main] 已加载相机标定参数" << std::endl;
            } else {
                std::cout << "[Main] 未找到相机标定文件，将使用默认参数" << std::endl;
            }

            // 创建显示窗口
            cv::namedWindow("Camera", cv::WINDOW_NORMAL);
            cv::namedWindow("Processing Steps", cv::WINDOW_NORMAL);

            std::cout << "\n=== 开始摄像头模式 ===" << std::endl;
            std::cout << "按'q'键退出程序" << std::endl;

            processCameraStream(camera, detector);
        }
        else if (mode == "--image") {
            if (argc < 3) {
                std::cerr << "[Main] 错误：未指定图片路径" << std::endl;
                printUsage();
                return -1;
            }

            std::string imagePath(argv[2]);
            
            // 创建篮筐检测器实例
            std::cout << "[Main] 创建篮筐检测器..." << std::endl;
            HoopDetector detector(5, 1, 0.7);

            // 尝试加载相机参数
            cv::FileStorage fs("camera_params.yml", cv::FileStorage::READ);
            if (fs.isOpened()) {
                cv::Mat camera_matrix, dist_coeffs;
                fs["camera_matrix"] >> camera_matrix;
                fs["distortion_coefficients"] >> dist_coeffs;
                detector.setCameraParams(camera_matrix, dist_coeffs);
                std::cout << "[Main] 已加载相机标定参数" << std::endl;
            } else {
                std::cout << "[Main] 未找到相机标定文件，将使用默认参数" << std::endl;
            }

            // 创建显示窗口
            cv::namedWindow("Image", cv::WINDOW_NORMAL);
            cv::namedWindow("Processing Steps", cv::WINDOW_NORMAL);

            std::cout << "\n=== 开始图片处理模式 ===" << std::endl;
            
            if (!processImage(imagePath, detector)) {
                std::cerr << "[Main] 图片处理失败" << std::endl;
                return -1;
            }
        }
        else if (mode == "--calibrate") {
            if (argc != 6) {
                std::cerr << "[Main] 错误: 标定模式参数不正确" << std::endl;
                printUsage();
                return -1;
            }

            int board_width = std::stoi(argv[2]);
            int board_height = std::stoi(argv[3]);
            float square_size = std::stof(argv[4]);
            std::string image_dir(argv[5]);

            CameraCalibrator calibrator;
            if (!calibrator.calibrateFromImages(image_dir, board_width, board_height, square_size)) {
                std::cerr << "[Main] 标定失败" << std::endl;
                return -1;
            }

            if (!calibrator.saveCalibrationResult("camera_params.yml")) {
                std::cerr << "[Main] 保存标定结果失败" << std::endl;
                return -1;
            }
        }
        else if (mode == "--verify") {
            if (argc != 4) {
                std::cerr << "[Main] 错误: 验证模式参数不正确" << std::endl;
                printUsage();
                return -1;
            }

            std::string calib_file(argv[2]);
            std::string test_image(argv[3]);

            CameraCalibrator calibrator;
            if (!calibrator.verifyCalibration(calib_file, test_image)) {
                std::cerr << "[Main] 验证失败" << std::endl;
                return -1;
            }
        }
        else if (mode == "--save") {
            std::string save_dir = (argc > 2) ? argv[2] : "saved_images";
            
            ImageSaver saver(save_dir);
            if (!saver.run()) {
                std::cerr << "[Main] 图片保存模式运行失败" << std::endl;
                return -1;
            }
        }
        else {
            std::cerr << "[Main] 未知的运行模式: " << mode << std::endl;
            printUsage();
            return -1;
        }

        std::cout << "[Main] 程序结束，开始清理资源..." << std::endl;
        cv::destroyAllWindows();
        std::cout << "[Main] 程序正常退出" << std::endl;

    } catch (const cv::Exception& e) {
        std::cerr << "[Main] OpenCV错误: " << e.what() << std::endl;
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "[Main] 发生错误: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "[Main] 发生未知错误!" << std::endl;
        return 1;
    }

    return 0;
} 