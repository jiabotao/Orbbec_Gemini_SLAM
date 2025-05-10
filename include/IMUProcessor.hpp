#pragma once
#include "IMUFrameQueueContainer.hpp"
#include "IMU.hpp"
#include <stdlib.h>

class IMUProcessor {
    public:
        void IMUCallBack(IMUFrameQueueContainer& imuFrameQueueContainer);
};