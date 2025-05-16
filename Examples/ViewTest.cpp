#include <libobsensor/ObSensor.hpp>
#include "utils.hpp"

int main() {
        // Create a pipeline with default device
    ob::Pipeline pipe;
    // Configure which streams to enable or disable for the Pipeline by creating a Config
    std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();
    // Get the ir_left camera configuration list
    std::shared_ptr<ob::StreamProfileList> irLeftProfiles = pipe.getStreamProfileList(OB_SENSOR_IR_LEFT);
    std::shared_ptr<ob::StreamProfile> irLeftProfile = irLeftProfiles->getProfile(26);
    config->enableStream(irLeftProfile->as<ob::VideoStreamProfile>());
    std::shared_ptr<ob::StreamProfileList> irRightProfiles = pipe.getStreamProfileList(OB_SENSOR_IR_RIGHT);
    std::shared_ptr<ob::StreamProfile> irRightProfile = irRightProfiles->getProfile(26);
    config->enableStream(irRightProfile->as<ob::VideoStreamProfile>());
    pipe.start(config);
    while(true) {
        // Wait for up to 100ms for a frameset in blocking mode.
        std::shared_ptr<ob::FrameSet> frameSet = pipe.waitForFrames(200);
        if(frameSet == nullptr) {
            continue;
        }

        // Get the data of left and right IR
        std::shared_ptr<ob::Frame> leftFrame  = frameSet->getFrame(OB_FRAME_IR_LEFT);
        std::shared_ptr<ob::Frame> rightFrame = frameSet->getFrame(OB_FRAME_IR_RIGHT);
        if(leftFrame == nullptr || rightFrame == nullptr) {
            continue;
        }
        
        std::shared_ptr<ob::VideoFrame> leftIRvideoFrame = leftFrame->as<ob::VideoFrame>();
        std::shared_ptr<ob::VideoFrame> rightIRvideoFrame = rightFrame->as<ob::VideoFrame>();
        std::cout << "left_time: " << leftIRvideoFrame->timeStamp() << ", right_time: " << rightIRvideoFrame->timeStamp() << std::endl;
    }


    return 0;
}