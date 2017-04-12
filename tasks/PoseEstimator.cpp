/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "PoseEstimator.hpp"
#include <uwv_kalman_filters/PoseUKF.hpp>
#include <pose_estimation/StreamAlignmentVerifier.hpp>
#include <pose_estimation/GravitationalModel.hpp>
#include <base-logging/Logging.hpp>
#include "uwv_kalman_filtersTypes.hpp"

using namespace uwv_kalman_filters;

typedef PoseUKF::WState FilterState;

PoseEstimator::PoseEstimator(std::string const& name)
    : PoseEstimatorBase(name)
{
}

PoseEstimator::PoseEstimator(std::string const& name, RTT::ExecutionEngine* engine)
    : PoseEstimatorBase(name, engine)
{
}

PoseEstimator::~PoseEstimator()
{
}

void PoseEstimator::body_effortsTransformerCallback(const base::Time &ts, const ::base::commands::LinearAngular6DCommand &body_efforts_sample)
{
    // create effort measurement
    PoseUKF::BodyEffortsMeasurement measurement;
    measurement.mu = body_efforts_sample.linear;
    measurement.cov = cov_body_efforts;

    try
    {
        // apply linear body effort measurement
        pose_filter->integrateMeasurement(measurement);
    }
    catch(const std::runtime_error& e)
    {
        LOG_ERROR_S << "Failed to integrate body effort measurements: " << e.what();
    }
}

void PoseEstimator::dvl_velocity_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &dvl_velocity_samples_sample)
{
    // receive sensor to body transformation
    Eigen::Affine3d dvlInIMU;
    if (!_dvl2imu.get(ts, dvlInIMU))
    {
        LOG_ERROR_S << "skip, couldn't receive a valid dvl-in-imu transformation sample!";
        new_state = MISSING_TRANSFORMATION;
        return;
    }

    if(dvl_velocity_samples_sample.hasValidVelocity() && dvl_velocity_samples_sample.hasValidVelocityCovariance())
    {
        base::Vector3d velocity = dvlInIMU.rotation() * dvl_velocity_samples_sample.velocity;
        velocity -= pose_filter->getRotationRate().cross(dvlInIMU.translation());

        // apply new velocity measurement
        PoseUKF::Velocity measurement;
        measurement.mu = velocity;
        measurement.cov = dvlInIMU.rotation() * dvl_velocity_samples_sample.cov_velocity * dvlInIMU.rotation().transpose();

        try
        {
            pose_filter->integrateMeasurement(measurement);
        }
        catch(const std::runtime_error& e)
        {
            LOG_ERROR_S << "Failed to integrate DVL measurement: " << e.what();
        }
    }
    else
        LOG_INFO_S << "DVL measurement contains NaN's, it will be skipped!";
}

void PoseEstimator::imu_sensor_samplesTransformerCallback(const base::Time &ts, const ::base::samples::IMUSensors &imu_sensor_samples_sample)
{
    // prediction step
    predictionStep(ts);

    // apply new gyro measurement
    PoseUKF::RotationRate rotation_rate;
    rotation_rate.mu = imu_sensor_samples_sample.gyro;
    rotation_rate.cov = cov_angular_velocity;

    try
    {
        pose_filter->integrateMeasurement(rotation_rate);
    }
    catch(const std::runtime_error& e)
    {
        LOG_ERROR_S << "Failed to integrate angular velocity measurement: " << e.what();
    }

    // apply new acceleration measurement
    PoseUKF::Acceleration acceleration;
    acceleration.mu = imu_sensor_samples_sample.acc;
    acceleration.cov = cov_acceleration;

    try
    {
        pose_filter->integrateMeasurement(acceleration);
    }
    catch(const std::runtime_error& e)
    {
        LOG_ERROR_S << "Failed to integrate acceleration measurement: " << e.what();
    }
}

void PoseEstimator::pressure_sensor_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &pressure_sensor_samples_sample)
{
    // receive sensor to IMU transformation
    Eigen::Affine3d pressureSensorInIMU;
    if (!_pressure_sensor2imu.get(ts, pressureSensorInIMU))
    {
        LOG_ERROR_S << "skip, couldn't receive a valid pressure-sensor-in-imu transformation sample!";
        new_state = MISSING_TRANSFORMATION;
        return;
    }

    Eigen::Matrix<double, 1, 1> altitude;
    altitude << pressure_sensor_samples_sample.position.z() - pressureSensorInIMU.translation().z();

    // apply altitude measurement
    PoseUKF::Z_Position measurement;
    measurement.mu = altitude;
    measurement.cov = pressure_sensor_samples_sample.cov_position.bottomRightCorner(1,1);

    try
    {
        pose_filter->integrateMeasurement(measurement);
    }
    catch(const std::runtime_error& e)
    {
        LOG_ERROR_S << "Failed to integrate altitude measurement: " << e.what();
    }
}

void PoseEstimator::xy_position_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &xy_position_samples_sample)
{
    // apply xy measurement
    PoseUKF::XY_Position measurement;
    measurement.mu = nav_in_nwu_2d * (xy_position_samples_sample.position.head<2>() + imu_in_body.translation().head<2>());
    measurement.cov = nav_in_nwu_2d.linear() * xy_position_samples_sample.cov_position.topLeftCorner(2,2) * nav_in_nwu_2d.linear().transpose();

    try
    {
        pose_filter->integrateMeasurement(measurement);
    }
    catch(const std::runtime_error& e)
    {
        LOG_ERROR_S << "Failed to integrate 2D position measurement: " << e.what();
    }
}

void PoseEstimator::gps_samplesTransformerCallback(const base::Time& ts, const gps_base::Solution& gps_samples_sample)
{
    // receive GPS to IMU transformation
    Eigen::Affine3d gpsInIMU;
    if (!_gps2imu.get(ts, gpsInIMU))
    {
        LOG_ERROR_S << "skip, couldn't receive a valid gps-in-imu transformation sample!";
        new_state = MISSING_TRANSFORMATION;
        return;
    }

    // assuming longitude and latitude in gps_base::Solution are in degree
    double latitude = base::Angle::deg2Rad(gps_samples_sample.latitude);
    double longitude = base::Angle::deg2Rad(gps_samples_sample.longitude);

    // apply gps measurement
    PoseUKF::GeographicPosition measurement;
    measurement.mu << latitude, longitude;
    measurement.cov << std::pow(gps_samples_sample.deviationLatitude, 2.), 0.,
                       0., std::pow(gps_samples_sample.deviationLongitude, 2.);

    try
    {
        pose_filter->integrateMeasurement(measurement, gpsInIMU.translation());
    }
    catch(const std::runtime_error& e)
    {
        LOG_ERROR_S << "Failed to integrate GPS measurement: " << e.what();
    }
}

void PoseEstimator::predictionStep(const base::Time& sample_time)
{
    try
    {
        pose_filter->predictionStepFromSampleTime(sample_time);
    }
    catch(const std::runtime_error& e)
    {
        LOG_ERROR_S << "Failed to execute prediction step: " << e.what();
        LOG_ERROR_S << "Skipping prediction step.";
    }
}

bool PoseEstimator::initializeFilter(const base::samples::RigidBodyState& initial_rbs,
                                     const PoseUKFConfig& filter_config,
                                     const uwv_dynamic_model::UWVParameters& model_parameters,
                                     const Eigen::Affine3d& imu_in_body,
                                     const Eigen::Affine3d& nav_in_nwu)
{
    if(!initial_rbs.hasValidPosition() || !initial_rbs.hasValidPositionCovariance()
        || !initial_rbs.hasValidOrientation() || !initial_rbs.hasValidOrientationCovariance())
    {
        LOG_ERROR_S << "Undefined values in initial pose!";
        return false;
    }

    if(model_parameters.damping_matrices.size() < 2)
    {
        LOG_ERROR_S << "The damping matrices must have at least two elements!";
        return false;
    }

    PoseUKF::State initial_state;
    initial_state.position = TranslationType(nav_in_nwu * (initial_rbs.position + imu_in_body.translation()));
    initial_state.orientation = RotationType(MTK::SO3<double>(nav_in_nwu.rotation() * initial_rbs.orientation));
    initial_state.velocity = VelocityType(Eigen::Vector3d::Zero());
    initial_state.acceleration = AccelerationType(Eigen::Vector3d::Zero());
    initial_state.bias_gyro = BiasType(filter_config.rotation_rate.bias_offset);
    initial_state.bias_acc = BiasType(filter_config.acceleration.bias_offset);
    Eigen::Matrix<double, 1, 1> gravity;
    gravity(0) = pose_estimation::GravitationalModel::WGS_84(filter_config.location.latitude, filter_config.location.altitude);
    initial_state.gravity = GravityType(gravity);
    initial_state.lin_damping.block(0,0,2,2) = model_parameters.damping_matrices[0].block(0,0,2,2);
    initial_state.lin_damping.block(0,2,2,1) = model_parameters.damping_matrices[0].block(0,5,2,1);
    initial_state.quad_damping.block(0,0,2,2) = model_parameters.damping_matrices[1].block(0,0,2,2);
    initial_state.quad_damping.block(0,2,2,1) = model_parameters.damping_matrices[1].block(0,5,2,1);

    PoseUKF::Covariance initial_state_cov = PoseUKF::Covariance::Zero();
    MTK::subblock(initial_state_cov, &FilterState::position) = nav_in_nwu.linear() * initial_rbs.cov_position * nav_in_nwu.linear().transpose();
    MTK::subblock(initial_state_cov, &FilterState::orientation) = nav_in_nwu.linear() * initial_rbs.cov_orientation * nav_in_nwu.linear().transpose();
    MTK::subblock(initial_state_cov, &FilterState::velocity) = Eigen::Matrix3d::Identity(); // velocity is unknown at the start
    MTK::subblock(initial_state_cov, &FilterState::acceleration) = Eigen::Matrix3d::Identity(); // acceleration is unknown at the start
    MTK::subblock(initial_state_cov, &FilterState::bias_gyro) = filter_config.rotation_rate.bias_instability.cwiseAbs2().asDiagonal();
    MTK::subblock(initial_state_cov, &FilterState::bias_acc) = filter_config.acceleration.bias_instability.cwiseAbs2().asDiagonal();
    Eigen::Matrix<double, 1, 1> gravity_var;
    gravity_var << pow(0.05, 2.); // give the gravity model a sigma of 5 cm/s^2 at the start
    MTK::subblock(initial_state_cov, &FilterState::gravity) = gravity_var;
    MTK::subblock(initial_state_cov, &FilterState::lin_damping) = filter_config.model_noise_parameters.lin_damping_instability.cwiseAbs2().asDiagonal();
    MTK::subblock(initial_state_cov, &FilterState::quad_damping) = filter_config.model_noise_parameters.quad_damping_instability.cwiseAbs2().asDiagonal();

    PoseUKF::PoseUKFParameter filter_parameter;
    filter_parameter.imu_in_body = imu_in_body.translation();
    filter_parameter.acc_bias_tau = filter_config.acceleration.bias_tau;
    filter_parameter.gyro_bias_tau = filter_config.rotation_rate.bias_tau;
    filter_parameter.lin_damping_tau = filter_config.model_noise_parameters.lin_damping_tau;
    filter_parameter.quad_damping_tau = filter_config.model_noise_parameters.quad_damping_tau;

    pose_filter.reset(new PoseUKF(initial_state, initial_state_cov, filter_config.location,
                                  model_parameters, filter_parameter));
    return true;
}

bool PoseEstimator::setProcessNoise(const PoseUKFConfig& filter_config, double imu_delta_t)
{
    PoseUKF::Covariance process_noise_cov = PoseUKF::Covariance::Zero();
    // Euler integration error position: (1/6/4 * jerk_max * dt^3)^2
    // assuming max jerk is 4*sigma devide by 4
    MTK::subblock(process_noise_cov, &FilterState::position) = 1.5 * (std::pow(imu_delta_t, 4.0) * ((1./6.) * 0.25 * filter_config.max_jerk).cwiseAbs2()).asDiagonal();
    LOG_DEBUG_S << "(sigma/delta_t)^2 position:\n" << MTK::subblock(process_noise_cov, &FilterState::position);
    // Euler integration error velocity: (1/2/4 * jerk_max * dt^2)^2
    MTK::subblock(process_noise_cov, &FilterState::velocity) = 1.5 * (std::pow(imu_delta_t, 2.0) * (0.5 * 0.25 * filter_config.max_jerk).cwiseAbs2()).asDiagonal();
    LOG_DEBUG_S << "(sigma/delta_t)^2 velocity:\n" << MTK::subblock(process_noise_cov, &FilterState::velocity);
    // Euler integration error acceleration: (1/4 * jerk_max * dt)^2
    MTK::subblock(process_noise_cov, &FilterState::acceleration) = (0.25 * filter_config.max_jerk).cwiseAbs2().asDiagonal();
    LOG_DEBUG_S << "(sigma/delta_t)^2 acceleration:\n" << MTK::subblock(process_noise_cov, &FilterState::acceleration);
    MTK::subblock(process_noise_cov, &FilterState::orientation) = filter_config.rotation_rate.randomwalk.cwiseAbs2().asDiagonal();
    MTK::subblock(process_noise_cov, &FilterState::bias_gyro) = (2. / (filter_config.rotation_rate.bias_tau * imu_delta_t)) *
                                        filter_config.rotation_rate.bias_instability.cwiseAbs2().asDiagonal();
    MTK::subblock(process_noise_cov, &FilterState::bias_acc) = (2. / (filter_config.acceleration.bias_tau * imu_delta_t)) *
                                        filter_config.acceleration.bias_instability.cwiseAbs2().asDiagonal();
    Eigen::Matrix<double, 1, 1> gravity_noise;
    gravity_noise << 1.e-12; // add a tiny bit of noise only for numeric stability
    MTK::subblock(process_noise_cov, &FilterState::gravity) = gravity_noise;
    MTK::subblock(process_noise_cov, &FilterState::lin_damping) = (2. / (filter_config.model_noise_parameters.lin_damping_tau * imu_delta_t)) *
                                        filter_config.model_noise_parameters.lin_damping_instability.cwiseAbs2().asDiagonal();
    MTK::subblock(process_noise_cov, &FilterState::quad_damping) = (2. / (filter_config.model_noise_parameters.quad_damping_tau * imu_delta_t)) *
                                        filter_config.model_noise_parameters.quad_damping_instability.cwiseAbs2().asDiagonal();
    pose_filter->setProcessNoiseCovariance(process_noise_cov);

    return true;
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See PoseEstimator.hpp for more detailed
// documentation about them.

bool PoseEstimator::configureHook()
{
    if (! PoseEstimatorBase::configureHook())
        return false;

    // get IMU to body transformation
    if(!_imu2body.get(base::Time(), imu_in_body))
    {
        LOG_ERROR_S << "Failed to get IMU pose in body frame. Note that this has to be a static transformation!";
        return false;
    }
    if(!imu_in_body.linear().isApprox(Eigen::Matrix3d::Identity()))
    {
        LOG_ERROR_S << "The IMU frame can't be rotated with respect to the body frame!";
        LOG_ERROR_S << "This is currently not supported by the filter.";
        return false;
    }

    // get navigation in NWU aligned frame
    if(!_navigation2navigation_nwu.get(base::Time(), nav_in_nwu))
    {
        LOG_ERROR_S << "Failed to get navigation in NWU frame. Note that this has to be a static transformation!";
        return false;
    }
    base::Vector3d nav_in_nwu_euler = base::getEuler(base::Orientation(nav_in_nwu.linear()));
    if(nav_in_nwu_euler.y() != 0. || nav_in_nwu_euler.z() != 0.)
    {
        LOG_ERROR_S << "The navigation frame can only be rotated with respect to the z axis in the NWU frame!";
        return false;
    }
    nwu_in_nav = nav_in_nwu.inverse();
    nav_in_nwu_2d = Eigen::Affine2d(Eigen::Rotation2Dd(nav_in_nwu_euler.x()));
    nav_in_nwu_2d.translation() = nwu_in_nav.translation().head<2>();

    // initialize filter
    if(!initializeFilter(_initial_state.value(), _filter_config.value(), _model_parameters.value(), imu_in_body, nav_in_nwu))
        return false;

    // set process noise
    if(!setProcessNoise(_filter_config.value(), _imu_sensor_samples_period.value()))
        return false;

    pose_filter->setMaxTimeDelta(_max_time_delta.get());

    // compute measurement covariances
    double sqrt_delta_t = sqrt(_imu_sensor_samples_period.value());
    Eigen::Vector3d rotation_rate_std = (1./sqrt_delta_t) * _filter_config.value().rotation_rate.randomwalk;
    Eigen::Vector3d acceleration_std = (1./sqrt_delta_t) * _filter_config.value().acceleration.randomwalk;
    cov_angular_velocity = rotation_rate_std.cwiseAbs2().asDiagonal();
    cov_acceleration = acceleration_std.cwiseAbs2().asDiagonal();
    cov_body_efforts = (1./_body_efforts_period.value()) * _filter_config.value().model_noise_parameters.body_efforts_std.cwiseAbs2().asDiagonal();

    // setup stream alignment verifier
    verifier.reset(new pose_estimation::StreamAlignmentVerifier());
    verifier->setVerificationInterval(20.0);
    verifier->setDropRateWarningThreshold(0.5);
    verifier->setDropRateCriticalThreshold(1.0);
    streams_with_alignment_failures = 0;
    streams_with_critical_alignment_failures = 0;

    last_sample_time = base::Time();

    last_state = PRE_OPERATIONAL;
    new_state = RUNNING;

    return true;
}
bool PoseEstimator::startHook()
{
    if (! PoseEstimatorBase::startHook())
        return false;
    return true;
}
void PoseEstimator::updateHook()
{
    new_state = RUNNING;
    PoseEstimatorBase::updateHook();

    // check stream alignment status
    verifier->verifyStreamAlignerStatus(_transformer.getStreamAlignerStatus(), streams_with_alignment_failures, streams_with_critical_alignment_failures);
    if(streams_with_alignment_failures > 0)
        new_state = TRANSFORMATION_ALIGNMENT_FAILURES;
    if(streams_with_critical_alignment_failures > 0)
        exception(CRITICAL_ALIGNMENT_FAILURE);

    // write estimated body state
    PoseUKF::State current_state;
    PoseUKF::Covariance state_cov;
    base::Time current_sample_time = pose_filter->getLastMeasurementTime();
    if(current_sample_time > last_sample_time && pose_filter->getCurrentState(current_state, state_cov))
    {
        /* Transform filter state from IMU in NWU aligned navigation frame to
         * body in navigation frame */
        base::samples::RigidBodyState pose_sample;
        pose_sample.position = nwu_in_nav * (Eigen::Vector3d(current_state.position) - imu_in_body.translation());
        pose_sample.orientation = nwu_in_nav.rotation() * current_state.orientation;
        pose_sample.angular_velocity = pose_filter->getRotationRate();
        pose_sample.velocity = nwu_in_nav.rotation() * Eigen::Vector3d(current_state.velocity) - pose_sample.orientation * pose_sample.angular_velocity.cross(imu_in_body.translation());
        pose_sample.cov_position = nwu_in_nav.linear() * MTK::subblock(state_cov, &FilterState::position) * nwu_in_nav.linear().transpose();
        pose_sample.cov_orientation = nwu_in_nav.linear() * MTK::subblock(state_cov, &FilterState::orientation) * nwu_in_nav.linear().transpose();
        pose_sample.cov_angular_velocity = cov_angular_velocity;
        pose_sample.cov_velocity = nwu_in_nav.linear() * MTK::subblock(state_cov, &FilterState::velocity) * nwu_in_nav.linear().transpose();
        pose_sample.time = current_sample_time;
        pose_sample.targetFrame = _navigation_frame.value();
        pose_sample.sourceFrame = _body_frame.value();
        _pose_samples.write(pose_sample);

        SecondaryStates secondary_states;
        secondary_states.acceleration = current_state.acceleration;
        secondary_states.cov_acceleration = MTK::subblock(state_cov, &FilterState::acceleration);
        secondary_states.bias_gyro = current_state.bias_gyro;
        secondary_states.cov_bias_gyro = MTK::subblock(state_cov, &FilterState::bias_gyro);
        secondary_states.bias_acc = current_state.bias_acc;
        secondary_states.cov_bias_acc = MTK::subblock(state_cov, &FilterState::bias_acc);
        secondary_states.gravity = current_state.gravity(0);
        secondary_states.var_gravity = MTK::subblock(state_cov, &FilterState::gravity)(0);
        secondary_states.lin_damping = current_state.lin_damping.cwiseAbs();
        secondary_states.cov_lin_damping_diag = MTK::subblock(state_cov, &FilterState::lin_damping).diagonal();
        secondary_states.quad_damping = current_state.quad_damping.cwiseAbs();
        secondary_states.cov_quad_damping_diag = MTK::subblock(state_cov, &FilterState::quad_damping).diagonal();
        secondary_states.time = current_sample_time;
        _secondary_states.write(secondary_states);

        last_sample_time = current_sample_time;
    }

    // write task state if it has changed
    if(last_state != new_state)
    {
        last_state = new_state;
        state(new_state);
    }
}
void PoseEstimator::errorHook()
{
    PoseEstimatorBase::errorHook();
}
void PoseEstimator::stopHook()
{
    PoseEstimatorBase::stopHook();
}
void PoseEstimator::cleanupHook()
{
    PoseEstimatorBase::cleanupHook();
}
