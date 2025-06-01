#include <libobsensor/ObSensor.hpp>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <System.h>

std::mutex framesetMutex;
std::mutex imuFrameMutex;

int main(int argc, char **argv)
{
    if (argc < 3 || argc > 4) {
        cerr << endl
             << "Usage: ./stereo_inertial_realsense_D435i path_to_vocabulary path_to_settings (trajectory_file_name)"
             << endl;
        return 1;
    }

    string file_name;

    if (argc == 4) {
        file_name = string(argv[argc - 1]);
    }

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
    std::shared_ptr<ob::Pipeline> imuPipeline(new ob::Pipeline(device));

    std::shared_ptr<ob::StreamProfileList> Left_IR_ProfileList = pipeline->getStreamProfileList(OB_SENSOR_IR_LEFT);

    std::shared_ptr<ob::VideoStreamProfile> Left_IR_Profile = Left_IR_ProfileList->getProfile(0)->as<ob::VideoStreamProfile>();
    int width = Left_IR_Profile->width();
    int height = Left_IR_Profile->height();
    int format = Left_IR_Profile->format();
    int fps = Left_IR_Profile->fps();
    //打印支持 Left_IR_Profile 数据流信息，包括图像的宽和高
    std::cout << "width: " << width << " height: " << height << " format: " << format << " fps: " << fps << std::endl;

    config->enableVideoStream(OB_STREAM_IR_LEFT,width,height,5,OB_FORMAT_Y8);
    config->enableVideoStream(OB_STREAM_IR_RIGHT,width,height,5,OB_FORMAT_Y8);
    config->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);

    pipeline->start(config, [&](std::shared_ptr<ob::FrameSet> frameSet) {
        std::lock_guard<std::mutex> lock(framesetMutex);
        std::shared_ptr<ob::Frame> left_ir_frame_raw = frameSet->getFrame(OB_FRAME_IR_LEFT);
        std::shared_ptr<ob::VideoFrame> left_ir_frame = left_ir_frame_raw->as<ob::VideoFrame>();

        std::shared_ptr<ob::Frame> right_ir_frame_raw = frameSet->getFrame(OB_FRAME_IR_RIGHT);
        std::shared_ptr<ob::VideoFrame> right_ir_frame = left_ir_frame_raw->as<ob::VideoFrame>();
       
        auto left_ir_data  = left_ir_frame->getData();
        auto right_ir_data = right_ir_frame->getData();


        cv::Mat imLeftCV  = cv::Mat(cv::Size(width, height), CV_8U, (void*)(left_ir_data), cv::Mat::AUTO_STEP);
        cv::Mat imRightCV = cv::Mat(cv::Size(width, height), CV_8U, (void*)(right_ir_data), cv::Mat::AUTO_STEP);
        std::cout << left_ir_frame->getFormat() << std::endl;
        
    });

    std::cout <<  "start configure imu" << std::endl;

    std::shared_ptr<ob::Config> imu_config = std::make_shared<ob::Config>();
    // 开启 加速度 流.
    imu_config->enableAccelStream();
    // 开启 角速度 流.
    imu_config->enableGyroStream();
    // Only FrameSet that contains all types of data frames will be output.
    imu_config->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);

    // Start the pipeline with config.
    imuPipeline->start(imu_config, [&](std::shared_ptr<ob::FrameSet> frameSet) {
        std::lock_guard<std::mutex> lockImu(imuFrameMutex);
        auto accelFrameRaw    = frameSet->getFrame(OB_FRAME_ACCEL);
        auto accelFrame       = accelFrameRaw->as<ob::AccelFrame>();
        auto accelIndex       = accelFrame->getIndex();
        auto accelTimeStampUs = accelFrame->getTimeStampUs();
        auto accelTemperature = accelFrame->getTemperature();
        auto accelType        = accelFrame->getType();
        if(accelIndex % 10 == 0) {  // print information every 50 frames.
            ob_accel_value accelValue = accelFrame->getValue();
            std::cout <<  "accelValue: " << accelValue.x<<" , " << accelValue.y <<" , " << accelValue.z << std::endl;
        }

        auto gyroFrameRaw    = frameSet->getFrame(OB_FRAME_GYRO);
        auto gyroFrame       = gyroFrameRaw->as<ob::GyroFrame>();
        auto gyroIndex       = gyroFrame->getIndex();
        auto gyroTimeStampUs = gyroFrame->getTimeStampUs();
        auto gyroTemperature = gyroFrame->getTemperature();
        auto gyroType        = gyroFrame->getType();
        if(gyroIndex % 10 == 0) {  // print information every 50 frames.
            ob_gyro_value gyroValue = gyroFrame->getValue();
            std::cout <<  "gyroValue: " << argv[1]<<" , " << argv[2] <<" , " << gyroValue.z << std::endl;
        }
    });

     // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_STEREO, true, 0, file_name);

    std::cin.get();

    return 0;
}