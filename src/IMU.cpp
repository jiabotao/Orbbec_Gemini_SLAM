#include "IMU.hpp"
#include <iostream>

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
    }
}