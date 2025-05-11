#include <sophus/so3.hpp>
#include <iostream>
#include "IMU.hpp"


namespace ORB_SLAM3
{
    namespace IMU
    {
        void Bias::CopyFrom(Bias &bias)
        {
            bias_accel_x = bias.bias_accel_x;
            bias_accel_y = bias.bias_accel_y;
            bias_accel_z = bias.bias_accel_y;
            bias_gyro_x = bias.bias_gyro_x;
            bias_gyro_y = bias.bias_gyro_y;
            bias_gyro_z = bias.bias_gyro_z;
        }

        std::ostream &operator<<(std::ostream &out, const Bias &bias)
        {
            if (bias.bias_gyro_x > 0)
                out << " ";
            out << bias.bias_gyro_x << ",";
            if (bias.bias_gyro_y > 0)
                out << " ";
            out << bias.bias_gyro_y << ",";
            if (bias.bias_gyro_z > 0)
                out << " ";
            out << bias.bias_gyro_z << ",";
            if (bias.bias_accel_x > 0)
                out << " ";
            out << bias.bias_accel_x << ",";
            if (bias.bias_accel_y > 0)
                out << " ";
            out << bias.bias_accel_y << ",";
            if (bias.bias_accel_z > 0)
                out << " ";
            out << bias.bias_accel_z;

            return out;
        }

        /**
         * 此函数需要#include <Eigen/SVD>
         * 在计算机视觉和机器人领域中，旋转矩阵通常用于表示物体的旋转姿态。
         * 旋转矩阵需要满足正交性和行列式为 1的条件。
         * 然而，由于数值计算误差的存在，在进行多次矩阵运算后，旋转矩阵可能会逐渐失去正交性，从而影响旋转表示的准确性。
         * 因此，需要对旋转矩阵进行归一化处理，使其重新满足正交性条件。
         * 行列式为 1 的正交矩阵属于特殊正交群 SO (n)，表示纯旋转。
         * */
        Eigen::Matrix3f NormalizeRotation(const Eigen::Matrix3f &R){
            Eigen::JacobiSVD<Eigen::Matrix3f> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
            return svd.matrixU() * svd.matrixV().transpose();
        }

        Eigen::Matrix3f RightJacobianSO3(const float &x, const float &y, const float &z)
        {
            Eigen::Matrix3f I;
            I.setIdentity();
            const float d2 = x*x+y*y+z*z;
            const float d = sqrt(d2);
            Eigen::Vector3f v;
            v << x, y, z;
            Eigen::Matrix3f W = Sophus::SO3f::hat(v);
            if(d<eps) {
                return I;
            }
            else {
                return I - W*(1.0f-cos(d))/d2 + W*W*(d-sin(d))/(d2*d);
            }
        }

        Eigen::Matrix3f RightJacobianSO3(const Eigen::Vector3f &v)
        {
            return RightJacobianSO3(v(0),v(1),v(2));
        }

        Eigen::Matrix3f InverseRightJacobianSO3(const float &x, const float &y, const float &z)
        {
            Eigen::Matrix3f I;
            I.setIdentity();
            const float d2 = x*x+y*y+z*z;
            const float d = sqrt(d2);
            Eigen::Vector3f v;
            v << x, y, z;
            Eigen::Matrix3f W = Sophus::SO3f::hat(v);

            if(d<eps) {
                return I;
            }
            else {
                return I + W/2 + W*W*(1.0f/d2 - (1.0f+cos(d))/(2.0f*d*sin(d)));
            }
        }

        Eigen::Matrix3f InverseRightJacobianSO3(const Eigen::Vector3f &v)
        {
            return InverseRightJacobianSO3(v(0),v(1),v(2));
        }

        IntegratedRotation::IntegratedRotation(const Eigen::Vector3f &angVel, const Bias &imuBias, const float &time) {
            const float x = (angVel(0)-imuBias.bias_gyro_x)*time;
            const float y = (angVel(1)-imuBias.bias_gyro_y)*time;
            const float z = (angVel(2)-imuBias.bias_gyro_z)*time;
        
            const float d2 = x*x+y*y+z*z;
            const float d = sqrt(d2);
        
            Eigen::Vector3f v;
            v << x, y, z;
            Eigen::Matrix3f W = Sophus::SO3f::hat(v);
            if(d<eps)
            {
                deltaR = Eigen::Matrix3f::Identity() + W;
                rightJ = Eigen::Matrix3f::Identity();
            }
            else
            {
                deltaR = Eigen::Matrix3f::Identity() + W*sin(d)/d + W*W*(1.0f-cos(d))/d2;
                rightJ = Eigen::Matrix3f::Identity() - W*(1.0f-cos(d))/d2 + W*W*(d-sin(d))/(d2*d);
            }
        }
    }
}