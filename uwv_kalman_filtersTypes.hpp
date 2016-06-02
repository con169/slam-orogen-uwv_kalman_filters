#ifndef uwv_kalman_filters_TYPES_HPP
#define uwv_kalman_filters_TYPES_HPP

#include <vector>
#include <base/Eigen.hpp>
#include <base/Float.hpp>

namespace uwv_kalman_filters
{

struct VelocityProcessNoise
{
    base::Matrix3d velocity_noise;
    double depth_noise;

    VelocityProcessNoise() : velocity_noise(base::Matrix3d::Constant(base::unknown<double>())),
                             depth_noise(base::unknown<double>()) {}
};

}

#endif


