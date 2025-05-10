#include "IMUFrameQueueContainer.hpp"

// 静态方法，用于获取单例实例
IMUFrameQueueContainer& IMUFrameQueueContainer::getInstance() {
    static IMUFrameQueueContainer instance;
    return instance;
}

// 向Accel队列入队元素
void IMUFrameQueueContainer::enqueueToAccelQueue(const std::shared_ptr<ob::Frame>& frame) {
    //std::cout << "推送ACCEL数据。\n\r" << std::endl;
    uint64_t timeStamp = frame->timeStamp();
    uint64_t index = frame->index();
    std::shared_ptr<ob::AccelFrame> accelFrame = frame->as<ob::AccelFrame>();
    OBAccelValue value = accelFrame->value();
    std::cout << "Accel Frame: {"
    << "  tsp = " << timeStamp << ""
    << "  temperature = " << accelFrame->temperature() << ""
    << "  gyro.x = " << value.x << " rad/s"
    << ""
    << "  gyro.y = " << value.y << " rad/s"
    << ""
    << "  gyro.z = " << value.z << " rad/s"
    << ""
    << "}\n\r" << std::endl;
    ACCEL_Frame_Queue.push(frame);
}

// 从Accel队列出队元素
bool IMUFrameQueueContainer::dequeueFromAccelQueue(std::shared_ptr<ob::Frame>& frame) {
    if (ACCEL_Frame_Queue.empty()) {
        return false;
    }
    frame = ACCEL_Frame_Queue.front();
    ACCEL_Frame_Queue.pop();
    return true;
}

// 向队列 2 入队元素
void IMUFrameQueueContainer::enqueueToGYROQueue(const std::shared_ptr<ob::Frame>& frame) {
    //std::cout << "推送GYRO数据。\n\r" << std::endl;
    std::shared_ptr<ob::GyroFrame> gyroFrame = frame->as<ob::GyroFrame>();
    uint64_t timeStamp = frame->timeStamp();
    OBGyroValue value = gyroFrame->value();
    std::cout << "Gyro Frame: {"
    << "  tsp = " << timeStamp << ""
    << "  temperature = " << gyroFrame->temperature() << ""
    << "  gyro.x = " << value.x << " rad/s"
    << ""
    << "  gyro.y = " << value.y << " rad/s"
    << ""
    << "  gyro.z = " << value.z << " rad/s"
    << ""
    << "}\n\r" << std::endl;
    GYRO_Frame_Queue.push(frame);
}

// 从队列 2 出队元素
bool IMUFrameQueueContainer::dequeueFromGYROQueue(std::shared_ptr<ob::Frame>& frame) {
    if (GYRO_Frame_Queue.empty()) {
        return false;
    }
    frame = GYRO_Frame_Queue.front();
    GYRO_Frame_Queue.pop();
    return true;
}

// 向队列 3 入队元素
void IMUFrameQueueContainer::enqueueToIMUQueue(const std::shared_ptr<ob::Frame>& frame) {
    IMU_Frame_Queue.push(frame);
}

// 从队列 3 出队元素
bool IMUFrameQueueContainer::dequeueFromIMUQueue(std::shared_ptr<ob::Frame>& frame) {
    if (IMU_Frame_Queue.empty()) {
        return false;
    }
    frame = IMU_Frame_Queue.front();
    IMU_Frame_Queue.pop();
    return true;
}

// 检查队列 1 是否为空
bool IMUFrameQueueContainer::IS_ACCEL_Frame_Queue_Empty() const {
    return ACCEL_Frame_Queue.empty();
}

// 检查队列 2 是否为空
bool IMUFrameQueueContainer::IS_GYRO_Frame_Queue_Empty() const {
    return GYRO_Frame_Queue.empty();
}

// 检查队列 3 是否为空
bool IMUFrameQueueContainer::IS_IMU_Frame_Queue_Empty() const {
    return IMU_Frame_Queue.empty();
}

// 获取队列 1 的大小
size_t IMUFrameQueueContainer::Get_ACCEL_Frame_QueueSize() const {
    return ACCEL_Frame_Queue.size();
}

// 获取队列 2 的大小
size_t IMUFrameQueueContainer::GET_GYRO_Frame_QueueSize() const {
    return GYRO_Frame_Queue.size();
}

// 获取队列 3 的大小
size_t IMUFrameQueueContainer::GET_IMU_Frame_QueueSize() const {
    return IMU_Frame_Queue.size();
}