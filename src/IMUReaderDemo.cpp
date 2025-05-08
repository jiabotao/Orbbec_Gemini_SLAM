#include <iostream>
#include <mutex>
#include <libobsensor/ObSensor.hpp>
#include "IMUFrameQueueContainer.hpp"
#include "utils.hpp"

#define ESC 27
std::mutex pushMutex;

int  main(int argc, char **argv) try {
    std::cout << "SDK version: " << ob::Version::getMajor() << "." << ob::Version::getMinor() << "." << ob::Version::getPatch() << std::endl;
    ob::Context ctx;
    std::shared_ptr<ob::DeviceList> deviceList = ctx.queryDeviceList();

    if(deviceList->deviceCount() == 0) {
        std::cerr << "未找到设备!" << std::endl;
        return -1;
    }

    std::shared_ptr<ob::Device> device = deviceList->getDevice(0);
    std::shared_ptr<ob::Sensor> gyroSensor  = nullptr;
    std::shared_ptr<ob::Sensor> accelSensor = nullptr;
    std::shared_ptr<ob::SensorList> sensorList = device->getSensorList();
    
    // 打印传感器数量
    try{
        uint32_t sensorCount = sensorList->count();
        std::cout << "传感器数量: " << sensorCount << std::endl;
    }catch(ob::Error &e){
        std::cerr << "传感器获取错误" << std::endl;
        exit(EXIT_FAILURE);
    }

    IMUFrameQueueContainer& imuFrameQueueContainer = IMUFrameQueueContainer::getInstance();

    // 获取角速度传感器,推送到GYROQueue
    try{
        gyroSensor = sensorList->getSensor(OB_SENSOR_GYRO);
        if(gyroSensor){
            std::cout << "成功获取到IMU(GYRO)传感器。" << std::endl;
            std::shared_ptr<ob::StreamProfileList> profiles = gyroSensor->getStreamProfileList();
            // 使用默认配置打开加速度传感器stream
            std::shared_ptr<ob::StreamProfile> profile = profiles->getProfile(OB_PROFILE_DEFAULT);
            gyroSensor->start(profile, [&](std::shared_ptr<ob::Frame> frame){
                std::unique_lock<std::mutex> lk(pushMutex);
                imuFrameQueueContainer.enqueueToGYROQueue(frame);
            });
        }
    }catch(ob::Error &e){
        std::cerr << "此设备不支持IMU(GYRO)传感器" << std::endl;
        exit(EXIT_FAILURE);
    }

    // 获取加速度传感器，推送到ACCELQueue
    try{
        accelSensor = sensorList->getSensor(OB_SENSOR_ACCEL);
        if(accelSensor){
            std::cout << "成功获取到IMU(ACCEL)传感器。" << std::endl;
            std::shared_ptr<ob::StreamProfileList> profiles = accelSensor->getStreamProfileList();
            // 使用默认配置打开加速度传感器stream
            std::shared_ptr<ob::StreamProfile> profile = profiles->getProfile(OB_PROFILE_DEFAULT);
            accelSensor->start(profile, [&](std::shared_ptr<ob::Frame> frame) {
                std::unique_lock<std::mutex> lk(pushMutex);
                imuFrameQueueContainer.enqueueToAccelQueue(frame);
            });
        }
        
    }catch(ob::Error &e){
        std::cerr << "此设备不支持IMU(ACCEL)传感器" << std::endl;
        exit(EXIT_FAILURE);
    }

    while(true) {
        // 捕获键盘输入，如果是ESC则退出程序
        int key = getch();
        if(key == ESC)
            break;
    }
    // 关闭传感器Stream
    if(gyroSensor) {
        gyroSensor->stop();
    }
    if(accelSensor) {
        accelSensor->stop();
    }
    return 0;

}catch(ob::Error &e) {
    std::cerr << "function:" << e.getName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.getMessage() << "\ntype:" << e.getExceptionType() << std::endl;
    exit(EXIT_FAILURE);
}

