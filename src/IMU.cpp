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
         * 通过奇异值分解并重构矩阵，可以得到一个新的矩阵，该矩阵是正交的，并且尽可能接近原始矩阵 \(\mathbf{R}\)。
         * */
        Eigen::Matrix3f NormalizeRotation(const Eigen::Matrix3f &R){
            /**
             * svd 是 Eigen::JacobiSVD<Eigen::Matrix3f> 类的一个对象实例。
             * 通过这个对象，我们可以访问奇异值分解的结果，像奇异值、左奇异向量矩阵 U 和右奇异向量矩阵 V 等。
             * 这行代码的作用是对矩阵 R 进行奇异值分解，并将分解结果存储在 svd 对象中，方便后续使用。
             */
            Eigen::JacobiSVD<Eigen::Matrix3f> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
            return svd.matrixU() * svd.matrixV().transpose();
        }

        /**
         * 计算特殊正交群SO(3)右雅可比矩阵的函数
         * 特殊正交群SO(3)常被用来表示三维空间中的旋转，而右雅可比矩阵在处理旋转的线性近似和误差传播等问题时非常重要。
         * x, y, z：这三个参数是三维向量v= [x, y, z]的三个分量，该向量表示旋转的李代数元素，对应于旋转轴和旋转角度的一种表示方式。
         * 用于处理旋转的线性近似和误差传播
         */
        Eigen::Matrix3f RightJacobianSO3(const float &x, const float &y, const float &z)
        {
            Eigen::Matrix3f I;
            I.setIdentity();// 创建一个 3x3 的单位矩阵
            const float d2 = x*x+y*y+z*z; // 计算向量 v 的模长的平方
            const float d = sqrt(d2); //计算向量 v 的模长
            Eigen::Vector3f v;
            v << x, y, z;  // 将 x, y, z 组合成一个三维向量 v
            Eigen::Matrix3f W = Sophus::SO3f::hat(v); // 计算向量 v 的反对称矩阵 W
            if(d < eps) { 
                return I; // 如果向量 v 的模长小于一个阈值 eps
            }
            else {
                return I - W*(1.0f-cos(d))/d2 + W*W*(d-sin(d))/(d2*d); // 使用完整公式计算右雅可比矩阵
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

        /**
         * 根据给定的角速度、IMU 偏置和时间间隔，计算单个陀螺仪测量值积分后的旋转矩阵 deltaR 以及右雅可比矩阵 rightJ。
         */
        IntegratedRotation::IntegratedRotation(const Eigen::Vector3f &angVel, const Bias &imuBias, const float &time) {
            // 计算去偏置后的角速度在时间间隔内的旋转量
            const float x = (angVel(0)-imuBias.bias_gyro_x)*time;
            const float y = (angVel(1)-imuBias.bias_gyro_y)*time;
            const float z = (angVel(2)-imuBias.bias_gyro_z)*time;
        
            // 计算旋转量的平方和以及模长
            const float d2 = x*x+y*y+z*z;
            const float d = sqrt(d2);
        
            // 将旋转量存储为三维向量
            Eigen::Vector3f v;
            v << x, y, z;
            // 计算反对称矩阵 W
            Eigen::Matrix3f W = Sophus::SO3f::hat(v);
            // 根据旋转量的模长判断是否接近零
            if(d<eps)
            {
                // 当旋转量很小时，使用一阶近似计算旋转矩阵和右雅可比矩阵
                deltaR = Eigen::Matrix3f::Identity() + W;
                rightJ = Eigen::Matrix3f::Identity();
            }
            else
            {
                // 当旋转量较大时，使用完整的公式计算旋转矩阵和右雅可比矩阵
                deltaR = Eigen::Matrix3f::Identity() + W*sin(d)/d + W*W*(1.0f-cos(d))/d2;
                rightJ = Eigen::Matrix3f::Identity() - W*(1.0f-cos(d))/d2 + W*W*(d-sin(d))/(d2*d);
            }
        }
    }
}