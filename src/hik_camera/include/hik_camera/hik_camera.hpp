#pragma once
#include <opencv2/opencv.hpp>
#include "MvCameraControl.h"

class HikCamera {
public:
    // 构造函数不再使用默认实现，需要初始化handle
    HikCamera();
    ~HikCamera();

    // 打开相机
    bool openCamera();
    
    // 关闭相机
    void closeCamera();
    
    // 获取一帧图像
    bool getFrame(cv::Mat& frame);
    
    // 设置相机参数
    bool setExposureTime(float exposureTime);
    bool setGain(float gain);
    
    // 获取相机参数
    bool getExposureTime(float& exposureTime);
    bool getGain(float& gain);

private:
    void* handle;  // 相机句柄
    unsigned char* pData_ = nullptr;  // 图像数据缓存
    bool isOpen_ = false;  // 相机是否打开
    
    // 将原始数据转换为OpenCV格式
    bool convertToMat(unsigned char* pData, cv::Mat& frame);
    
    // 清理资源
    void cleanup();
}; 