#ifndef uwv_kalman_filters_TYPES_HPP
#define uwv_kalman_filters_TYPES_HPP

#include <vector>
#include <base/Eigen.hpp>
#include <base/Float.hpp>
#include <base/Time.hpp>

namespace uwv_kalman_filters
{

struct VelocityProcessNoise
{
    base::Matrix3d velocity_noise;
    double depth_noise;

    VelocityProcessNoise() : velocity_noise(base::Matrix3d::Constant(base::unknown<double>())),
                             depth_noise(base::unknown<double>()) {}
};

struct PoseEstimatorProcessNoise
{
    base::Vector3d position_diag;
    base::Vector3d acceleration_diag;

    PoseEstimatorProcessNoise() : position_diag(base::Vector3d::Constant(base::unknown<double>())),
                                  acceleration_diag(base::Vector3d::Constant(base::unknown<double>())) {}
};

struct SecondaryStates
{
    base::Time time;
    base::Vector3d acceleration;
    base::Matrix3d cov_acceleration;
    base::Vector3d bias_gyro;
    base::Matrix3d cov_bias_gyro;
    base::Vector3d bias_acc;
    base::Matrix3d cov_bias_acc;
    double gravity;
    double var_gravity;
};

}

#endif


