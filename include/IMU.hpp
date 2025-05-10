#ifndef IMU_H
#define IMU_H

#include <opencv2/core/core.hpp>
#include <Eigen/Core>
// #include <Eigen/Geometry>
// #include <Eigen/Dense>

namespace ORB_SLAM3
{
    namespace IMU
    {
        const float GRAVITY_VALUE=9.7936;

        //IMU measurement (gyro, accelerometer and timestamp)
        class Point
        {
            public:
                Point(const float &acc_x, const float &acc_y, const float &acc_z,
                        const float &ang_vel_x, const float &ang_vel_y, const float &ang_vel_z,
                        const double &timestamp): a(acc_x,acc_y,acc_z), w(ang_vel_x,ang_vel_y,ang_vel_z), t(timestamp){}
                Point(const cv::Point3f Acc, const cv::Point3f Gyro, const double &timestamp):
                    a(Acc.x,Acc.y,Acc.z), w(Gyro.x,Gyro.y,Gyro.z), t(timestamp){}
            public:
                Eigen::Vector3f a;
                Eigen::Vector3f w;
                double t;
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            };
    }
        
}

#endif // IMU_H