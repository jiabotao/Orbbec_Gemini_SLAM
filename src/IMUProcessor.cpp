#include <iostream>
#include <thread>
#include <atomic>
#include <functional>
#include "IMUProcessor.hpp"


void IMUProcessor::IMUCallBack(IMUFrameQueueContainer& imuFrameQueueContainer ) {
    while (true) {
        if(imuFrameQueueContainer.Get_ACCEL_Frame_QueueSize()>0 && imuFrameQueueContainer.GET_GYRO_Frame_QueueSize() > 0 )
        {
            std::shared_ptr<ob::Frame> accel_frame = imuFrameQueueContainer.dequeueFromAccelQueue();
            std::shared_ptr<ob::Frame>  gyro_frame = imuFrameQueueContainer.dequeueFromGYROQueue();

            std::cout << "Thread is running..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1)); // 避免CPU占用过高
        }
    }
    std::cout << "Thread stopped." << std::endl;
}