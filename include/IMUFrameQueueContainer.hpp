#ifndef IMUFRAMEQUEUECONTAINER_H
#define IMUFRAMEQUEUECONTAINER_H

#include <queue>
#include <memory>
#include <libobsensor/ObSensor.hpp>

class IMUFrameQueueContainer {
private:
    std::queue<std::shared_ptr<ob::Frame>> ACCEL_Frame_Queue;
    std::queue<std::shared_ptr<ob::Frame>> GYRO_Frame_Queue;
    // 将ACCEL_Frame_Queue和GYRO_Frame_Queue合并成一个队列，TimeStamp对齐后丢给后续流程
    std::queue<std::shared_ptr<ob::Frame>> IMU_Frame_Queue;

    // 构造函数私有化，防止外部实例化
    IMUFrameQueueContainer() = default;
    // 拷贝构造函数和赋值运算符私有化，防止复制
    IMUFrameQueueContainer(const IMUFrameQueueContainer&) = delete;
    IMUFrameQueueContainer& operator=(const IMUFrameQueueContainer&) = delete;
    
public:
    // 静态方法，用于获取单例实例
    static IMUFrameQueueContainer& getInstance();

    // 向队列 1 入队元素
    void enqueueToAccelQueue(const std::shared_ptr<ob::Frame>& frame);

    // 从队列 1 出队元素
    std::shared_ptr<ob::Frame> dequeueFromAccelQueue();

    // 向队列 2 入队元素
    void enqueueToGYROQueue(const std::shared_ptr<ob::Frame>& frame);

    // 从队列 2 出队元素
    std::shared_ptr<ob::Frame> dequeueFromGYROQueue();

    // 向队列 3 入队元素
    void enqueueToIMUQueue(const std::shared_ptr<ob::Frame>& frame);

    // 从队列 3 出队元素
    std::shared_ptr<ob::Frame> dequeueFromIMUQueue();

    // 检查队列 1 是否为空
    bool IS_ACCEL_Frame_Queue_Empty() const;

    // 检查队列 2 是否为空
    bool IS_GYRO_Frame_Queue_Empty() const;

    // 检查队列 3 是否为空
    bool IS_IMU_Frame_Queue_Empty() const;

    // 获取队列 1 的大小
    size_t Get_ACCEL_Frame_QueueSize() const;

    // 获取队列 2 的大小
    size_t GET_GYRO_Frame_QueueSize() const;

    // 获取队列 3 的大小
    size_t GET_IMU_Frame_QueueSize() const;
};

#endif // IMUFRAMEQUEUECONTAINER_H