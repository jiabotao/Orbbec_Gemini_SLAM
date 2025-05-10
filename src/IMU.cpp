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
    }
}