#include <iostream>
#include <mutex>
#include <libobsensor/ObSensor.hpp>
#include "IMUFrameQueueContainer.hpp"
#include "utils.hpp"
#include <thread>
#include <IMUProcessor.hpp>

#define ESC 27
std::mutex imu_mutex;

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

    // 获取角速度传感器,推送到GYROQueue;获取加速度传感器，推送到ACCELQueue
    try{
        gyroSensor = sensorList->getSensor(OB_SENSOR_GYRO);
        accelSensor = sensorList->getSensor(OB_SENSOR_ACCEL);

        if(gyroSensor && accelSensor){
            std::shared_ptr<ob::StreamProfileList> gyro_profiles = gyroSensor->getStreamProfileList();
            std::shared_ptr<ob::StreamProfileList> accel_profiles = accelSensor->getStreamProfileList();
            // 使用默认配置打开角速度传感器stream
            std::shared_ptr<ob::StreamProfile> gyro_profile = gyro_profiles->getProfile(OB_PROFILE_DEFAULT);
            // 使用默认配置打开加速度传感器stream
            std::shared_ptr<ob::StreamProfile> accel_profile = accel_profiles->getProfile(OB_PROFILE_DEFAULT);

            std::thread gyroThread([&]() {
                gyroSensor->start(gyro_profile, [&](std::shared_ptr<ob::Frame> frame) {
                    std::unique_lock<std::mutex> lk(imu_mutex);
                    imuFrameQueueContainer.enqueueToGYROQueue(frame);
                });
            });
            
            std::thread accelThread([&]() {
                accelSensor->start(accel_profile, [&](std::shared_ptr<ob::Frame> frame) {
                    std::unique_lock<std::mutex> lk(imu_mutex);
                    imuFrameQueueContainer.enqueueToAccelQueue(frame);
                });
            });
            
            // 等待两个线程完成启动
            gyroThread.join();
            accelThread.join();
        }
    }catch(ob::Error &e){
        std::cerr << "此设备不支持IMU(GYRO)传感器" << std::endl;
        exit(EXIT_FAILURE);
    }

    IMUProcessor imuProcessor;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::thread imuThread(&IMUProcessor::IMUCallBack, &imuProcessor, std::ref(imuFrameQueueContainer));
    if (imuThread.joinable()) {
        imuThread.join();
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

