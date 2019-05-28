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

struct BottomEstimatorParameters
{
    /* Standard deviation of the range measurements in m at 1Hz */
    double range_measurement_std;
    /* Maximum range of the DVL sensor in m */
    double dvl_max_range;
    /* Change of distance at 1m/s in m */
    double distance_noise;
    /* Change of normal angles at 1m/s in rad */
    double surface_normal_noise;
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
    base::MatrixXd inertia;
    base::VectorXd cov_inertia_diag;
    base::MatrixXd lin_damping;
    base::VectorXd cov_lin_damping_diag;
    base::MatrixXd quad_damping;
    base::VectorXd cov_quad_damping_diag;
    base::Vector2d water_velocity;
    base::Matrix2d cov_water_velocity;
    base::Vector2d water_velocity_below;
    base::Matrix2d cov_water_velocity_below;
    base::Vector2d bias_adcp;
    base::Matrix2d cov_bias_adcp;
    double water_density;
    double var_water_density;
};

}

#endif


