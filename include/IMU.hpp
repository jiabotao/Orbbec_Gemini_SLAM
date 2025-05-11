#ifndef IMU_H
#define IMU_H

#include <opencv2/core/core.hpp>
#include <Eigen/Core>
#include <boost/serialization/serialization.hpp>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <sophus/so3.hpp>

namespace ORB_SLAM3
{
    namespace IMU
    {
        const float GRAVITY_VALUE = 9.7936;
        const float eps = 1e-4;
        
        //Integration of 1 gyro measurement
        class IntegratedRotation
        {
        public:
            IntegratedRotation(){}
            IntegratedRotation(const Eigen::Vector3f &angVel, const Bias &imuBias, const float &time);

        public:
            float deltaT; //integration time
            Eigen::Matrix3f deltaR;
            Eigen::Matrix3f rightJ; // right jacobian
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };

        // IMU measurement (gyro, accelerometer and timestamp)
        class Point
        {
        public:
            Point(const float &accel_x, const float &accel_y, const float &accel_z,
                  const float &gyro_x, const float &gyro_y, const float &gyro_z,
                  const double &timestamp) : accel(accel_x, accel_y, accel_z),
                                             gyro(gyro_x, gyro_y, gyro_z),
                                             timestamp(timestamp) {}
            Point(const cv::Point3f Accel,
                  const cv::Point3f Gyro,
                  const double &timestamp) : accel(Accel.x, Accel.y, Accel.z),
                                             gyro(Gyro.x, Gyro.y, Gyro.z),
                                             timestamp(timestamp) {}

        public:
            Eigen::Vector3f accel;
            Eigen::Vector3f gyro;
            double timestamp;
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };

        // IMU biases (gyro and accel)
        class Bias
        {
            friend class boost::serialization::access;
            template <class Archive>
            void serialize(Archive &archive, const unsigned int version)
            {
                archive & bias_accel_x;
                archive & bias_accel_y;
                archive & bias_accel_y;

                archive & bias_gyro_x;
                archive & bias_gyro_y;
                archive & bias_gyro_z;
            }

        public:
            Bias() : bias_accel_x(0), bias_accel_y(0), bias_accel_z(0), bias_gyro_x(0), bias_gyro_y(0), bias_gyro_z(0) {}
            Bias(const float &bias_accel_x, const float &bias_accel_y, const float &bias_accel_z,
                 const float &bias_gyro_x, const float &bias_gyro_y, const float &bias_gyro_z) : bias_accel_x(bias_accel_x), bias_accel_y(bias_accel_y), bias_accel_z(bias_accel_z),
                                                                                                 bias_gyro_x(bias_gyro_x), bias_gyro_y(bias_gyro_y), bias_gyro_z(bias_gyro_z) {}
            void CopyFrom(Bias &bias);
            friend std::ostream &operator<<(std::ostream &out, const Bias &bias);

        public:
            float bias_accel_x, bias_accel_y, bias_accel_z;
            float bias_gyro_x, bias_gyro_y, bias_gyro_z;
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };

        Eigen::Matrix3f NormalizeRotation(const Eigen::Matrix3f &R);
        Eigen::Matrix3f RightJacobianSO3(const float &x, const float &y, const float &z);
        Eigen::Matrix3f RightJacobianSO3(const Eigen::Vector3f &v);
        Eigen::Matrix3f InverseRightJacobianSO3(const float &x, const float &y, const float &z);
        Eigen::Matrix3f InverseRightJacobianSO3(const Eigen::Vector3f &v);
    }


}

#endif // IMU_H