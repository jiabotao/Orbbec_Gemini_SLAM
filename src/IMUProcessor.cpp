#include <iostream>
#include <thread>
#include <atomic>
#include <functional>
#include "IMUProcessor.hpp"

void IMUProcessor::IMUCallBack(IMUFrameQueueContainer& imuFrameQueueContainer ) {
    while (true) {
        if(imuFrameQueueContainer.Get_ACCEL_Frame_QueueSize() > 0 && imuFrameQueueContainer.GET_GYRO_Frame_QueueSize() > 0 )
        {
            std::shared_ptr<ob::Frame> accel_frame = imuFrameQueueContainer.dequeueFromAccelQueue();
            std::shared_ptr<ob::Frame>  gyro_frame = imuFrameQueueContainer.dequeueFromGYROQueue();

            uint64_t accel_frame_timeStamp = accel_frame->timeStamp();
            uint64_t gyro_frame_timeStamp  = gyro_frame->timeStamp();

            uint64_t accel_frame_index = accel_frame->index();
            uint64_t gyro_frame_index = gyro_frame->index();
            if(accel_frame_index == gyro_frame_index){
               std::cout << "accel=" << accel_frame_timeStamp << ", gyro=" << gyro_frame_timeStamp << "\n";
            }
        }
    }
    std::cout << "Thread stopped." << std::endl;
}