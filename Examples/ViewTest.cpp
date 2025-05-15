#include <libobsensor/ObSensor.hpp>
#include "utils.hpp"

int main() {
    ob::Context ctx;
    std::shared_ptr<ob::DeviceList> deviceList = ctx.queryDeviceList();
    if(deviceList->deviceCount() == 0) {
        std::cerr << "未找到设备!" << std::endl;
        return -1;
    }
    std::shared_ptr<ob::Device> device = deviceList->getDevice(0);
    auto depthModeList = device->getDepthWorkModeList();
    std::cout << "depthModeList size: " << depthModeList->count() << std::endl;
    for(int i = 0; i < depthModeList->count(); i++) {
        //std::cout << "depthModeList[" << i << "]: " << (*depthModeList)[i].name << std::endl;
    }

    std::shared_ptr<ob::Pipeline> pipeline = std::make_shared<ob::Pipeline>(device);
    std::shared_ptr<ob::StreamProfileList> OB_SENSOR_IR_LEFT_ProfileList = pipeline->getStreamProfileList(OB_SENSOR_IR_LEFT);
    std::cout << "OB_SENSOR_IR_LEFT_ProfileList size: " << OB_SENSOR_IR_LEFT_ProfileList->count() << std::endl;
    for(int i = 0; i < OB_SENSOR_IR_LEFT_ProfileList->count(); i++) {
        auto colorProfile = OB_SENSOR_IR_LEFT_ProfileList->getProfile(i)->as<ob::VideoStreamProfile>();
        std::cout << "color profile: " << colorProfile->width() << "x" << colorProfile->height() << " @ " << colorProfile->fps() << "fps" << std::endl;
    }


    return 0;
}