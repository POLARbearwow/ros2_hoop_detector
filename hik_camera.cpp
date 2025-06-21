#include "hik_camera.hpp"
#include <iostream>

HikCamera::HikCamera() : handle(nullptr) {
    std::cout << "[HikCamera] 构造函数被调用" << std::endl;
}

HikCamera::~HikCamera() {
    std::cout << "[HikCamera] 析构函数被调用" << std::endl;
    closeCamera();
}

bool HikCamera::openCamera() {
    int nRet = MV_OK;

    // 初始化SDK
    std::cout << "[HikCamera] 初始化SDK..." << std::endl;
    nRet = MV_CC_Initialize();
    if (MV_OK != nRet) {
        std::cerr << "[HikCamera] Initialize SDK fail! nRet [0x" << std::hex << nRet << "]" << std::endl;
        return false;
    }

    // 枚举设备
    std::cout << "[HikCamera] 枚举设备..." << std::endl;
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet) {
        std::cerr << "[HikCamera] Enum Devices fail! nRet [0x" << std::hex << nRet << "]" << std::endl;
        return false;
    }
    
    if (stDeviceList.nDeviceNum == 0) {
        std::cerr << "[HikCamera] 未找到任何相机设备!" << std::endl;
        return false;
    }
    std::cout << "[HikCamera] 找到 " << stDeviceList.nDeviceNum << " 个设备" << std::endl;

    // 选择第一个设备
    std::cout << "[HikCamera] 创建相机句柄..." << std::endl;
    nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[0]);
    if (MV_OK != nRet) {
        std::cerr << "[HikCamera] Create Handle fail! nRet [0x" << std::hex << nRet << "]" << std::endl;
        return false;
    }

    // 打开设备
    std::cout << "[HikCamera] 打开设备..." << std::endl;
    nRet = MV_CC_OpenDevice(handle);
    if (MV_OK != nRet) {
        std::cerr << "[HikCamera] Open Device fail! nRet [0x" << std::hex << nRet << "]" << std::endl;
        return false;
    }

    // 获取相机参数
    MVCC_INTVALUE stParam;
    nRet = MV_CC_GetIntValue(handle, "Width", &stParam);
    if (MV_OK == nRet) {
        std::cout << "[HikCamera] 图像宽度: " << stParam.nCurValue << std::endl;
    }
    nRet = MV_CC_GetIntValue(handle, "Height", &stParam);
    if (MV_OK == nRet) {
        std::cout << "[HikCamera] 图像高度: " << stParam.nCurValue << std::endl;
    }

    // 设置手动曝光
    std::cout << "[HikCamera] 设置相机参数..." << std::endl;
    nRet = MV_CC_SetEnumValue(handle, "ExposureAuto", 0);
    if (MV_OK != nRet) {
        std::cerr << "[HikCamera] Set ExposureAuto fail! nRet [0x" << std::hex << nRet << "]" << std::endl;
        return false;
    }

    // 设置曝光时间
    nRet = MV_CC_SetFloatValue(handle, "ExposureTime", 3000.0f);
    if (MV_OK != nRet) {
        std::cerr << "[HikCamera] Set ExposureTime fail! nRet [0x" << std::hex << nRet << "]" << std::endl;
        return false;
    }

    // 设置手动增益
    nRet = MV_CC_SetEnumValue(handle, "GainAuto", 0);
    if (MV_OK != nRet) {
        std::cerr << "[HikCamera] Set GainAuto fail! nRet [0x" << std::hex << nRet << "]" << std::endl;
        return false;
    }

    // 设置增益值
    nRet = MV_CC_SetFloatValue(handle, "Gain", 23.9f);
    if (MV_OK != nRet) {
        std::cerr << "[HikCamera] Set Gain fail! nRet [0x" << std::hex << nRet << "]" << std::endl;
        return false;
    }

    // 设置像素格式为RGB8
    std::cout << "[HikCamera] 设置像素格式..." << std::endl;
    nRet = MV_CC_SetEnumValue(handle, "PixelFormat", PixelType_Gvsp_BGR8_Packed);
    if (MV_OK != nRet) {
        std::cerr << "[HikCamera] Set PixelFormat fail! nRet [0x" << std::hex << nRet << "]" << std::endl;
        return false;
    }

    // 开始取流
    std::cout << "[HikCamera] 开始取流..." << std::endl;
    nRet = MV_CC_StartGrabbing(handle);
    if (MV_OK != nRet) {
        std::cerr << "[HikCamera] Start Grabbing fail! nRet [0x" << std::hex << nRet << "]" << std::endl;
        return false;
    }

    return true;
}

bool HikCamera::getFrame(cv::Mat& frame) {
    if (!handle) {
        std::cerr << "[HikCamera] 相机句柄为空!" << std::endl;
        return false;
    }

    MV_FRAME_OUT stImageInfo = {0};
    memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT));

    // std::cout << "[HikCamera] 尝试获取图像..." << std::endl;
    int nRet = MV_CC_GetImageBuffer(handle, &stImageInfo, 1000);
    if (nRet == MV_OK) {
        // std::cout << "[HikCamera] 成功获取图像，大小: " 
        //           << stImageInfo.stFrameInfo.nWidth << "x" 
        //           << stImageInfo.stFrameInfo.nHeight 
        //           << " 像素格式: " << stImageInfo.stFrameInfo.enPixelType << std::endl;

        if (stImageInfo.pBufAddr == nullptr) {
            std::cerr << "[HikCamera] 图像缓冲区指针为空!" << std::endl;
            return false;
        }

        try {
            cv::Mat rawData(stImageInfo.stFrameInfo.nHeight, 
                          stImageInfo.stFrameInfo.nWidth, 
                          CV_8UC3, 
                          stImageInfo.pBufAddr);
            
            // 复制数据
            rawData.copyTo(frame);

            // 检查复制后的图像
            if (frame.empty()) {
                std::cerr << "[HikCamera] 复制后的图像为空!" << std::endl;
                MV_CC_FreeImageBuffer(handle, &stImageInfo);
                return false;
            }

            // std::cout << "[HikCamera] 图像复制成功" << std::endl;
        }
        catch (const cv::Exception& e) {
            std::cerr << "[HikCamera] OpenCV错误: " << e.what() << std::endl;
            MV_CC_FreeImageBuffer(handle, &stImageInfo);
            return false;
        }
        catch (const std::exception& e) {
            std::cerr << "[HikCamera] 标准错误: " << e.what() << std::endl;
            MV_CC_FreeImageBuffer(handle, &stImageInfo);
            return false;
        }
        catch (...) {
            std::cerr << "[HikCamera] 未知错误!" << std::endl;
            MV_CC_FreeImageBuffer(handle, &stImageInfo);
            return false;
        }

        // 释放图像缓存
        nRet = MV_CC_FreeImageBuffer(handle, &stImageInfo);
        if (nRet != MV_OK) {
            std::cerr << "[HikCamera] Free Image Buffer fail! nRet [0x" << std::hex << nRet << "]" << std::endl;
            return false;
        }
        return true;
    }
    else {
        std::cerr << "[HikCamera] 获取图像失败! nRet [0x" << std::hex << nRet << "]" << std::endl;
        return false;
    }
}

void HikCamera::closeCamera() {
    if (handle) {
        std::cout << "[HikCamera] 停止取流..." << std::endl;
        MV_CC_StopGrabbing(handle);
        
        std::cout << "[HikCamera] 关闭设备..." << std::endl;
        MV_CC_CloseDevice(handle);
        
        std::cout << "[HikCamera] 销毁句柄..." << std::endl;
        MV_CC_DestroyHandle(handle);
        handle = nullptr;
    }
    std::cout << "[HikCamera] 终止SDK..." << std::endl;
    MV_CC_Finalize();
}

bool HikCamera::setExposureTime(float exposureTime) {
    if (!handle) return false;
    
    int nRet = MV_CC_SetFloatValue(handle, "ExposureTime", exposureTime);
    if (MV_OK != nRet) {
        std::cerr << "[HikCamera] 设置曝光时间失败! nRet [" << nRet << "]" << std::endl;
        return false;
    }
    return true;
}

bool HikCamera::setGain(float gain) {
    if (!handle) return false;
    
    int nRet = MV_CC_SetFloatValue(handle, "Gain", gain);
    if (MV_OK != nRet) {
        std::cerr << "[HikCamera] 设置增益失败! nRet [" << nRet << "]" << std::endl;
        return false;
    }
    return true;
}

bool HikCamera::getExposureTime(float& exposureTime) {
    if (!handle) return false;
    
    MVCC_FLOATVALUE stParam = {0};
    int nRet = MV_CC_GetFloatValue(handle, "ExposureTime", &stParam);
    if (MV_OK != nRet) {
        std::cerr << "[HikCamera] 获取曝光时间失败! nRet [" << nRet << "]" << std::endl;
        return false;
    }
    exposureTime = stParam.fCurValue;
    return true;
}

bool HikCamera::getGain(float& gain) {
    if (!handle) return false;
    
    MVCC_FLOATVALUE stParam = {0};
    int nRet = MV_CC_GetFloatValue(handle, "Gain", &stParam);
    if (MV_OK != nRet) {
        std::cerr << "[HikCamera] 获取增益失败! nRet [" << nRet << "]" << std::endl;
        return false;
    }
    gain = stParam.fCurValue;
    return true;
}

bool HikCamera::convertToMat(unsigned char* pData, cv::Mat& frame) {
    if (!pData) return false;

    // 假设图像格式为BGR8打包
    // 根据实际情况可能需要调整
    frame = cv::Mat(1200, 1920, CV_8UC3, pData);
    return true;
}

void HikCamera::cleanup() {
    if (pData_) {
        delete[] pData_;
        pData_ = nullptr;
    }
} 
 