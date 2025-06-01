#include <libobsensor/ObSensor.hpp>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <System.h>

std::mutex framesetMutex;
std::mutex imuFrameMutex;

std::deque<std::pair<uint64_t, cv::Mat>> left_ir_buffer;
std::deque<std::pair<uint64_t, cv::Mat>> right_ir_buffer;
std::deque<std::pair<uint64_t, ORB_SLAM3::IMU::Point>> imu_buffer;


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
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_STEREO, true, 0, file_name);

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
        ob_accel_value accelValue = accelFrame->getValue();
        // if(accelIndex % 10 == 0) {  // print information every 50 frames.
        //     ob_accel_value accelValue = accelFrame->getValue();
        //     std::cout <<  "accelValue: " << accelValue.x<<" , " << accelValue.y <<" , " << accelValue.z << "TimeStampUs" << accelTimeStampUs << std::endl;
        // }

        auto gyroFrameRaw    = frameSet->getFrame(OB_FRAME_GYRO);
        auto gyroFrame       = gyroFrameRaw->as<ob::GyroFrame>();
        auto gyroIndex       = gyroFrame->getIndex();
        auto gyroTimeStampUs = gyroFrame->getTimeStampUs();
        auto gyroTemperature = gyroFrame->getTemperature();
        auto gyroType        = gyroFrame->getType();
        OBGyroValue gyroValue = gyroFrame->getValue();
        //std::cout << "a_time: " << gyroTimeStampUs << std::endl;

        ORB_SLAM3::IMU::Point lastPoint(accelValue.x,accelValue.y, accelValue.z,
                                  gyroValue.x, gyroValue.y, gyroValue.z,
                                  gyroTimeStampUs*1e-6);

        imu_buffer.emplace_back(gyroTimeStampUs,lastPoint);
    });


    std::cout <<  "start configure ir" << std::endl;

    int width = 1280;
    int height = 720;
    int fps = 30;
    config->enableVideoStream(OB_STREAM_IR_LEFT,width,height,fps,OB_FORMAT_Y8);
    config->enableVideoStream(OB_STREAM_IR_RIGHT,width,height,fps,OB_FORMAT_Y8);
    config->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);

    pipeline->start(config, [&](std::shared_ptr<ob::FrameSet> frameSet) {
        std::lock_guard<std::mutex> lock(framesetMutex);
        std::shared_ptr<ob::Frame> left_ir_frame_raw = frameSet->getFrame(OB_FRAME_IR_LEFT);
        std::shared_ptr<ob::VideoFrame> left_ir_frame = left_ir_frame_raw->as<ob::VideoFrame>();

        std::shared_ptr<ob::Frame> right_ir_frame_raw = frameSet->getFrame(OB_FRAME_IR_RIGHT);
        std::shared_ptr<ob::VideoFrame> right_ir_frame = left_ir_frame_raw->as<ob::VideoFrame>();
        uint64_t left_ir_frame_TimeStampUs = left_ir_frame->getTimeStampUs();
        uint64_t right_ir_frame_TimeStampUs = right_ir_frame->getTimeStampUs();
       
        auto left_ir_data  = left_ir_frame->getData();
        auto right_ir_data = right_ir_frame->getData();


        cv::Mat imLeftCV  = cv::Mat(cv::Size(width, height), CV_8U, (void*)(left_ir_data), cv::Mat::AUTO_STEP);
        cv::Mat imRightCV = cv::Mat(cv::Size(width, height), CV_8U, (void*)(right_ir_data), cv::Mat::AUTO_STEP);
        //std::cout << "b_time: " << left_ir_frame_TimeStampUs << std::endl;

        left_ir_buffer.emplace_back(left_ir_frame_TimeStampUs,imLeftCV);
        right_ir_buffer.emplace_back(left_ir_frame_TimeStampUs,imRightCV);

        
    });

     // Create SLAM system. It initializes all system threads and gets ready to process frames.
    

    while (!SLAM.isShutDown()){
        if(right_ir_buffer.size() > 0 && left_ir_buffer.size()){
            auto [left_timestamp, left_img] = left_ir_buffer.front();
            auto [right_timestamp, right_img] = right_ir_buffer.front();
            left_ir_buffer.pop_front();
            right_ir_buffer.pop_front();
            //std::cout << "left_timestamp: " << left_timestamp << "   left_timestamp: "<< right_timestamp << std::endl;
            //SLAM.TrackStereo(imLeftCV, imRightCV, left_ir_frame_TimeStampUs, vector_imu);
            auto [imu_timestamp, imu_data]  = imu_buffer.front();
            vector<ORB_SLAM3::IMU::Point> vector_imu;
            while (true) {
                auto [imu_timestamp, imu_data]  = imu_buffer.front();
                if (imu_timestamp < right_timestamp) {
                    imu_buffer.pop_front();
                    vector_imu.push_back(imu_data);
                }else{
                    break;
                }
            }
            SLAM.TrackStereo(left_img, right_img, right_timestamp*1e-6, vector_imu);
        }
    }

    std::cin.get();

    return 0;
}