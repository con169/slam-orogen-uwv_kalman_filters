/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "VelocityProvider.hpp"
#include <uwv_filters/VelocityUKF.hpp>
#include <pose_estimation/EulerConversion.hpp>

using namespace uwv_filters;

VelocityProvider::VelocityProvider(std::string const& name)
    : VelocityProviderBase(name)
{
}

VelocityProvider::VelocityProvider(std::string const& name, RTT::ExecutionEngine* engine)
    : VelocityProviderBase(name, engine)
{
}

VelocityProvider::~VelocityProvider()
{
}

void VelocityProvider::dvl_velocity_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &dvl_velocity_samples_sample)
{
    // receive sensor to body transformation
    Eigen::Affine3d dvlInBody;
    if (!_dvl2body.get(ts, dvlInBody))
    {
        RTT::log(RTT::Error) << "skip, couldn't receive a valid dvl-in-body transformation sample!" << RTT::endlog();
        new_state = MISSING_TRANSFORMATION;
        return;
    }

    if(dvl_velocity_samples_sample.hasValidVelocity() && dvl_velocity_samples_sample.hasValidVelocityCovariance())
    {
        base::Vector3d velocity = dvlInBody.rotation() * dvl_velocity_samples_sample.velocity;
        if(base::isnotnan(current_angular_velocity) && !current_angular_velocity.isZero())
        {
            base::Vector3d euler_angle_velocity = base::getEuler(base::Orientation(Eigen::AngleAxisd(current_angular_velocity.norm(), current_angular_velocity.normalized())));
            velocity -= Eigen::Vector3d(euler_angle_velocity.z(), euler_angle_velocity.y(), euler_angle_velocity.x()).cross(dvlInBody.translation());
        }

        // add velocity measurement
        pose_estimation::Measurement measurement;
        measurement.time = ts;
        measurement.measurement_name = "dvl_measurement";
        measurement.integration = pose_estimation::StateMapping;
        measurement.mu = velocity;
        measurement.cov = dvlInBody.rotation() * dvl_velocity_samples_sample.cov_velocity * dvlInBody.rotation().transpose();
        measurement.state_mapping.resize(3);
        for(unsigned i = 0; i < 3; i++)
            measurement.state_mapping(i) = i;
        if(!pose_estimator->enqueueMeasurement(measurement))
            RTT::log(RTT::Error) << "Failed to add DVL measurement." << RTT::endlog();
    }
    else
        RTT::log(RTT::Error) << "DVL measurement contains NaN's, it will be skipped!" << RTT::endlog();
}

void VelocityProvider::imu_sensor_samplesTransformerCallback(const base::Time &ts, const ::base::samples::IMUSensors &imu_sensor_samples_sample)
{
    // receive sensor to body transformation
    Eigen::Affine3d imuInBody;
    if (!_imu2body.get(ts, imuInBody))
    {
        RTT::log(RTT::Error) << "skip, couldn't receive a valid imu-in-body transformation sample!" << RTT::endlog();
        new_state = MISSING_TRANSFORMATION;
        return;
    }

    if(_use_acceleration_samples.value())
    {
        if(base::isnotnan(imu_sensor_samples_sample.acc))
        {
            // enqueue new acceleration measurement
            pose_estimation::Measurement acc_measurement;
            acc_measurement.time = ts;
            acc_measurement.integration = pose_estimation::UserDefined;
            acc_measurement.measurement_name = VelocityUKF::acceleration_measurement;
            acc_measurement.mu = imuInBody.rotation() * imu_sensor_samples_sample.acc;
            acc_measurement.cov = _cov_acceleration.value();
            if(!pose_estimator->enqueueMeasurement(acc_measurement))
                RTT::log(RTT::Error) << "Failed to add acceleration measurement." << RTT::endlog();
        }
        else
            RTT::log(RTT::Error) << "Acceleration measurement contains NaN's, it will be skipped!" << RTT::endlog();
    }

    if(base::isnotnan(imu_sensor_samples_sample.gyro))
    {
        // enqueue new gyro measurement
        pose_estimation::Measurement gyro_measurement;
        gyro_measurement.time = ts;
        gyro_measurement.integration = pose_estimation::UserDefined;
        gyro_measurement.measurement_name = VelocityUKF::angular_velocity_measurement;
        current_angular_velocity = imuInBody.rotation() * imu_sensor_samples_sample.gyro;
        gyro_measurement.mu = current_angular_velocity;
        gyro_measurement.cov = _cov_angular_velocity.value();
        if(!pose_estimator->enqueueMeasurement(gyro_measurement))
            RTT::log(RTT::Error) << "Failed to add angular velocity measurement." << RTT::endlog();
    }
    else
        RTT::log(RTT::Error) << "Angular velocity measurement contains NaN's, it will be skipped!" << RTT::endlog();
}

void VelocityProvider::joint_samplesTransformerCallback(const base::Time &ts, const ::base::samples::Joints &joint_samples_sample)
{
    pose_estimation::Measurement measurement;
    measurement.time = ts;
    measurement.measurement_name = VelocityUKF::thruster_rpm_measurement;
    measurement.integration = pose_estimation::UserDefined;
    measurement.mu.resize(joint_samples_sample.elements.size());
    for(unsigned i = 0; i < joint_samples_sample.elements.size(); i++)
    {
        if(!base::isNaN(joint_samples_sample.elements[i].speed))
            measurement.mu(i) = (double)joint_samples_sample.elements[i].speed;
        else
        {
            RTT::log(RTT::Error) << "Thruster speed measurements contain NaN values! Measurement will be skipped." << RTT::endlog();
            return;
        }
    }
    // enqueue new measurement
    if(!pose_estimator->enqueueMeasurement(measurement))
        RTT::log(RTT::Error) << "Failed to add thruster speed measurements." << RTT::endlog();
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See VelocityProvider.hpp for more detailed
// documentation about them.

bool VelocityProvider::configureHook()
{
    if (! VelocityProviderBase::configureHook())
        return false;

    // setup initial state
    VelocityUKF::FilterState init_state;
    // linear velocity
    init_state.mu = Eigen::Vector3d::Zero();
    init_state.cov = 0.1 * base::Matrix3d::Identity();
    boost::shared_ptr<VelocityUKF> filter(new VelocityUKF(init_state));

    // set model parameters
    filter->setupMotionModel(_model_parameters.value());

    pose_estimator.reset(new pose_estimation::PoseEstimator(filter));

    // setup process noise
    const VelocityProcessNoise& process_noise = _process_noise.value();
    VelocityUKF::Covariance process_noise_cov = VelocityUKF::Covariance::Zero();

    if(base::isnotnan(process_noise.velocity_noise))
        process_noise_cov.block(0,0,3,3) = process_noise.velocity_noise;
    else
        process_noise_cov.block(0,0,3,3) = 0.001 * base::Matrix3d::Identity();

    pose_estimator->setProcessNoise(process_noise_cov);

    pose_estimator->setMaxTimeDelta(_max_time_delta.get());

    // setup stream alignment verifier
    verifier.reset(new pose_estimation::StreamAlignmentVerifier());
    verifier->setVerificationInterval(2.0);
    verifier->setDropRateThreshold(0.5);
    streams_with_alignment_failures = 0;

    current_angular_velocity = Eigen::Vector3d::Zero();

    return true;
}
bool VelocityProvider::startHook()
{
    if (! VelocityProviderBase::startHook())
        return false;
    return true;
}
void VelocityProvider::updateHook()
{
    VelocityProviderBase::updateHook();

    // integrate measurements
    try
    {
        pose_estimator->integrateMeasurements();
    }
    catch (std::runtime_error e)
    {
        RTT::log(RTT::Error) << "Failed to integrate measurements: " << e.what() << RTT::endlog();
    }

    // check stream alignment status
    verifier->verifyStreamAlignerStatus(_transformer.getStreamAlignerStatus(), streams_with_alignment_failures);
    if(streams_with_alignment_failures > 0)
        new_state = TRANSFORMATION_ALIGNMENT_FAILURES;

    // write estimated body state
    VelocityUKF::FilterState current_state;
    if(pose_estimator->getEstimatedState(current_state))
    {
        base::samples::RigidBodyState velocity_sample;
        velocity_sample.velocity = current_state.mu.block(0,0,3,1);
        Eigen::Vector3d angular_velocity;
        EulerConversion::eulerAngleVelocityToAngleAxis(current_state.mu.block(3,0,3,1), angular_velocity);
        velocity_sample.angular_velocity = angular_velocity;
        velocity_sample.cov_velocity = current_state.cov.block(0,0,3,3);
        velocity_sample.cov_angular_velocity = current_state.cov.block(3,3,3,3);
        velocity_sample.time = pose_estimator->getLastMeasurementTime();
        velocity_sample.targetFrame = _target_frame.value();
        velocity_sample.sourceFrame = _target_frame.value();
        _velocity_samples.write(velocity_sample);
    }

    // write task state if it has changed
    if(last_state != new_state)
    {
        last_state = new_state;
        state(new_state);
    }
}
void VelocityProvider::errorHook()
{
    VelocityProviderBase::errorHook();
}
void VelocityProvider::stopHook()
{
    VelocityProviderBase::stopHook();
}
void VelocityProvider::cleanupHook()
{
    VelocityProviderBase::cleanupHook();
}
