#pragma once
#include "IMUFrameQueueContainer.hpp"

class IMUProcessor {
public:
    void IMUCallBack(IMUFrameQueueContainer& imuFrameQueueContainer);
};