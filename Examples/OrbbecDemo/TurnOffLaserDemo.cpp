#include <libobsensor/ObSensor.hpp>
#include <mutex>

std::shared_ptr<ob::Context> ctx;
std::shared_ptr<ob::Device> device;
std::shared_ptr<ob::Pipeline> pipeline;
std::recursive_mutex deviceMutex;

int main()
{
    std::shared_ptr<ob::Context> ctx = std::make_shared<ob::Context>();
    std::shared_ptr<ob::DeviceList> devices = ctx->queryDeviceList();
    device = devices->getDevice(0);
    device->setBoolProperty(OB_PROP_LASER_CONTROL_INT, 0);
    device->setBoolProperty(OB_PROP_IR_AUTO_EXPOSURE_BOOL, 1);
    device->setIntProperty(OB_PROP_IR_AE_MAX_EXPOSURE_INT, 100000);

    return 0;

}