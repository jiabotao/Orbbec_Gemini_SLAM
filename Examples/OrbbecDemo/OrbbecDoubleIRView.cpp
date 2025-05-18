#include <libobsensor/ObSensor.hpp>
#include "utils.hpp"
#include <mutex>

#define ESC 27

void handleFrameset(std::shared_ptr<ob::FrameSet> frameset);

std::shared_ptr<ob::Context> ctx;
std::shared_ptr<ob::Config> config;
std::shared_ptr<ob::Device> device;

std::mutex frameMutex;

int main() 
{
    ctx = std::make_shared<ob::Context>();
    auto config = std::make_shared<ob::Config>();
    std::shared_ptr<ob::DeviceList> devices = ctx->queryDeviceList();
    device = devices->getDevice(0);
    device->setBoolProperty(OB_PROP_LASER_CONTROL_INT, 0);

    
    std::shared_ptr<ob::Pipeline> pipe = std::make_shared<ob::Pipeline>(device);
    if (!pipe) {
        std::cerr << "XXXXXXXXXXXXX." << std::endl;
        return 1;
    }
    std::shared_ptr<ob::StreamProfileList> irLeftProfiles  = pipe->getStreamProfileList(OB_SENSOR_IR_LEFT);
    std::shared_ptr<ob::StreamProfileList> irRightProfiles = pipe->getStreamProfileList(OB_SENSOR_IR_RIGHT);
    if(irLeftProfiles == nullptr && irRightProfiles == nullptr) 
    {
        std::cerr << "左IR或右IR配置有误." << std::endl;
        return 0;
    }
    //26号为 1280*800分辨率 5帧
    std::shared_ptr<ob::StreamProfile> irLeftProfile  = irLeftProfiles ->getProfile(0);
    config->enableStream(irLeftProfile->as<ob::VideoStreamProfile>());
    std::shared_ptr<ob::StreamProfile> irRightProfile = irRightProfiles->getProfile(0);
    config->enableStream(irRightProfile->as<ob::VideoStreamProfile>());
    pipe->start(config,handleFrameset);


    return 0;
}

void handleFrameset(std::shared_ptr<ob::FrameSet> frameset) {
    std::lock_guard<std::mutex> lock(frameMutex);
    std::cout << "333333333" << std::endl;
}