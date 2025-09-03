/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "BottomEstimator.hpp"

using namespace uwv_kalman_filters;

BottomEstimator::BottomEstimator(std::string const& name)
    : BottomEstimatorBase(name)
{
}

BottomEstimator::~BottomEstimator()
{
}

void BottomEstimator::dvl_bottom_trackingTransformerCallback(const base::Time &ts, const ::dvl_teledyne::BottomTracking &dvl_bottom_tracking_sample)
{
    if(!valid_body_pose)
        return;

    Eigen::Vector3d delta_position = Eigen::Vector3d::Zero();
    if(last_body_pose.matrix().allFinite())
    {
        delta_position = current_body_pose.translation() - last_body_pose.translation();
    }

    // set velocity
    double delta_t = (ts - bottom_filter->getLastMeasurementTime()).toSeconds();
    //double delta_t = 0;
    if(delta_t > bottom_filter->getMinTimeDelta())
        bottom_filter->setVelocity(delta_position / delta_t);

    // execute prediction step
    predictionStep(ts);

    // integrate DVL range measurements
    for(unsigned i = 0; i < 4; i++)
    {
        if(!base::isNaN(dvl_bottom_tracking_sample.range[i]))
        {
            Eigen::Affine3d transducer_in_aligned_body = current_body_pose.rotation() * transducer_in_body[i];

            BottomUKF::RangeMeasurement range_measurement;
            range_measurement.mu << dvl_bottom_tracking_sample.range[i];
            range_measurement.cov << range_measurement_var;

            bottom_filter->integrateMeasurement(range_measurement,
                                                transducer_in_aligned_body.rotation() * Eigen::Vector3d::UnitZ(),
                                                transducer_in_aligned_body.translation());
        }
    }

    last_body_pose = current_body_pose;
}

void BottomEstimator::pose_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &pose_samples_sample)
{
    valid_body_pose = true;
    current_body_pose = pose_samples_sample.getTransform();

    // integrate normal measurements in order to constrain the orientation on the top half of the unit sphere
    if((ts - normal_measurement_time).toSeconds() >= 1.0)
    {
        bottom_filter->integrateMeasurement(normal_measurement, normal_measurement_cov);
        normal_measurement_time = ts;
    }
}

void BottomEstimator::predictionStep(const base::Time& sample_time)
{
    try
    {
        bottom_filter->predictionStepFromSampleTime(sample_time);
    }
    catch(const std::runtime_error& e)
    {
        LOG_ERROR_S << "Failed to execute prediction step: " << e.what();
        LOG_ERROR_S << "Skipping prediction step.";
    }
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See BottomEstimator.hpp for more detailed
// documentation about them.

bool BottomEstimator::configureHook()
{
    if (! BottomEstimatorBase::configureHook())
        return false;

    if(!_dvl_transducer_02body.get(base::Time(), transducer_in_body[0]))
    {
        LOG_ERROR_S << "Couldn't get DVL transdicer 0 in body transformation!";
        return false;
    }
    if(!_dvl_transducer_12body.get(base::Time(), transducer_in_body[1]))
    {
        LOG_ERROR_S << "Couldn't get DVL transdicer 1 in body transformation!";
        return false;
    }
    if(!_dvl_transducer_22body.get(base::Time(), transducer_in_body[2]))
    {
        LOG_ERROR_S << "Couldn't get DVL transdicer 2 in body transformation!";
        return false;
    }
    if(!_dvl_transducer_32body.get(base::Time(), transducer_in_body[3]))
    {
        LOG_ERROR_S << "Couldn't get DVL transdicer 3 in body transformation!";
        return false;
    }

    // normal measurements in order to constrain the orientation on the top half of the unit sphere
    normal_measurement = MTK::S2<double>(Eigen::Vector3d::UnitZ());
    normal_measurement_cov = std::pow(M_PI_2, 2.0) * Eigen::Matrix2d::Identity();

    return true;
}
bool BottomEstimator::startHook()
{
    if (! BottomEstimatorBase::startHook())
        return false;

    config = _filter_config.rvalue();

    // setup initial state
    BottomUKF::State init_state;
    init_state.distance = DistanceType(config.dvl_max_range * 0.5);
    init_state.normal = NormalType(MTK::S2<double>(Eigen::Vector3d::UnitZ()));

    BottomUKF::Covariance state_cov;
    state_cov = BottomUKF::Covariance::Zero();
    Eigen::Matrix<double, 1, 1> distance_var;
    distance_var << std::pow(config.dvl_max_range * 0.5, 2.0);
    MTK::subblock(state_cov, &BottomUKF::State::distance) = distance_var;
    MTK::subblock(state_cov, &BottomUKF::State::normal) = std::pow(M_PI_4, 2.0) * Eigen::Matrix2d::Identity();

    bottom_filter.reset(new BottomUKF(init_state, state_cov));

    // setup process noise
    BottomUKF::Covariance process_noise_cov = BottomUKF::Covariance::Zero();
    distance_var << std::pow(config.distance_noise, 2.0);
    MTK::subblock(process_noise_cov, &BottomUKF::State::distance) = distance_var;
    MTK::subblock(process_noise_cov, &BottomUKF::State::normal) = std::pow(config.surface_normal_noise, 2.0) * Eigen::Matrix2d::Identity();
    bottom_filter->setProcessNoiseCovariance(process_noise_cov);

    bottom_filter->setMaxTimeDelta(_max_time_delta.get());

    range_measurement_var = (1./_dvl_bottom_tracking_period.rvalue()) * std::pow(config.range_measurement_std, 2.0);

    // setup stream alignment verifier
    verifier.reset(new pose_estimation::StreamAlignmentVerifier());
    verifier->setVerificationInterval(20.0);
    verifier->setDropRateWarningThreshold(0.5);
    verifier->setDropRateCriticalThreshold(1.0);
    streams_with_alignment_failures = 0;
    streams_with_critical_alignment_failures = 0;

    // reset state variables
    last_sample_time = base::Time();
    last_body_pose.matrix() = base::NaN<double>() * Eigen::Matrix4d::Ones();
    current_body_pose.matrix() = base::NaN<double>() * Eigen::Matrix4d::Ones();
    valid_body_pose = false;
    normal_measurement_time = base::Time();

    last_state = PRE_OPERATIONAL;
    new_state = RUNNING;

    return true;
}
void BottomEstimator::updateHook()
{
    new_state = RUNNING;
    BottomEstimatorBase::updateHook();

    // check stream alignment status
    verifier->verifyStreamAlignerStatus(_transformer.getStreamAlignerStatus(), streams_with_alignment_failures, streams_with_critical_alignment_failures);
    if(streams_with_alignment_failures > 0)
        new_state = TRANSFORMATION_ALIGNMENT_FAILURES;
    if(streams_with_critical_alignment_failures > 0)
        exception(CRITICAL_ALIGNMENT_FAILURE);

    // write estimated body state
    BottomUKF::State current_state;
    BottomUKF::Covariance state_cov;
    base::Time current_sample_time = bottom_filter->getLastMeasurementTime();
    if(current_sample_time > last_sample_time && bottom_filter->getCurrentState(current_state, state_cov))
    {
        base::samples::RigidBodyState ground_sample;
        ground_sample.position.z() = -1. * current_state.distance.value;
        ground_sample.position = ground_sample.position;
        ground_sample.cov_position.block(2,2,1,1) = MTK::subblock(state_cov, &BottomUKF::State::distance);
        ground_sample.orientation = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), current_state.normal.get_vect());
        ground_sample.cov_orientation.block(0,0,2,2) = MTK::subblock(state_cov, &BottomUKF::State::normal);
        ground_sample.time = current_sample_time;
        ground_sample.targetFrame = _body_frame.rvalue();
        ground_sample.sourceFrame = _source_frame.rvalue();
        _ground_samples.write(ground_sample);
        last_sample_time = current_sample_time;
    }

    // write task state if it has changed
    if(last_state != new_state)
    {
        last_state = new_state;
        state(new_state);
    }
}
void BottomEstimator::errorHook()
{
    BottomEstimatorBase::errorHook();
}
void BottomEstimator::stopHook()
{
    BottomEstimatorBase::stopHook();
}
void BottomEstimator::cleanupHook()
{
    BottomEstimatorBase::cleanupHook();
}
