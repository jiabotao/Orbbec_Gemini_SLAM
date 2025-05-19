#include <libobsensor/ObSensor.hpp>
#include <window.hpp>

int main()
{
    ob::Context context;
    std::shared_ptr<ob::DeviceList> deviceList = context.queryDeviceList();
    if(deviceList->getCount() < 1)
    {
        std::cerr << "没有找到设备" << std::endl;
        return 0;
    }
    std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();
    std::shared_ptr<ob::Device> device = deviceList->getDevice(0);

    device->setIntProperty(OB_PROP_LASER_CONTROL_INT, 0);

    std::shared_ptr<ob::Pipeline> pipeline(new ob::Pipeline(device));

    std::shared_ptr<ob::StreamProfileList> Left_IR_ProfileList = pipeline->getStreamProfileList(OB_SENSOR_IR_LEFT);

    std::shared_ptr<ob::VideoStreamProfile> Left_IR_Profile = Left_IR_ProfileList->getProfile(5)->as<ob::VideoStreamProfile>();
    int width = Left_IR_Profile->width();
    int height = Left_IR_Profile->height();
    int format = Left_IR_Profile->format();
    int fps = Left_IR_Profile->fps();
    //打印支持 Left_IR_Profile 数据流信息，包括图像的宽和高
    std::cout << "width: " << width << " height: " << height << " format: " << format << " fps: " << fps << std::endl;

    config->enableVideoStream(OB_STREAM_IR_LEFT,width,height,5,OB_FORMAT_Y8);
    
    pipeline->start(config);


    Window app("ColorViewer", width, height);
    while(app) {
        // Wait for up to 100ms for a frameset in blocking mode.
        auto frameSet = pipeline->waitForFrames(200);
        if(frameSet == nullptr) {
            continue;
        }

        // get color frame from frameset
        auto leftFrame  = frameSet->getFrame(OB_FRAME_IR_LEFT);

        // Render frameset in the window, only color frames are rendered here.
        app.addToRender(leftFrame);
    }

    // Stop the Pipeline, no frame data will be generated
    pipeline->stop();


    

    return 0;
}