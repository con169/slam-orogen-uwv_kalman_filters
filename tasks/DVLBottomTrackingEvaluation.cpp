/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "DVLBottomTrackingEvaluation.hpp"

using namespace uwv_kalman_filters;

DVLBottomTrackingEvaluation::DVLBottomTrackingEvaluation(std::string const& name, TaskCore::TaskState initial_state)
    : DVLBottomTrackingEvaluationBase(name, initial_state)
{
}

DVLBottomTrackingEvaluation::DVLBottomTrackingEvaluation(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : DVLBottomTrackingEvaluationBase(name, engine, initial_state)
{
}

DVLBottomTrackingEvaluation::~DVLBottomTrackingEvaluation()
{
}

bool DVLBottomTrackingEvaluation::evaluateBottomTrackingQuality(const dvl_teledyne::BottomTracking& bottom_tracking)
{
    // check NaNs
    if(base::isUnknown(bottom_tracking.velocity[0]) ||
        base::isUnknown(bottom_tracking.velocity[1]) ||
        base::isUnknown(bottom_tracking.velocity[2]) ||
        base::isUnknown(bottom_tracking.velocity[3]))
    {
        return false;
    }

    // check correlations
    if(bottom_tracking.correlation[0] < _min_correlation.value() ||
        bottom_tracking.correlation[1] < _min_correlation.value() ||
        bottom_tracking.correlation[2] < _min_correlation.value() ||
        bottom_tracking.correlation[3] < _min_correlation.value())
    {
        return false;
    }
    // check ping ratios
    if(_evaluate_ping_ratio.value() &&
       (bottom_tracking.good_ping_ratio[0] <= 0.0 ||
        bottom_tracking.good_ping_ratio[1] <= 0.0 ||
        bottom_tracking.good_ping_ratio[2] <= 0.0 ||
        bottom_tracking.good_ping_ratio[3] <= 0.0))
    {
        return false;
    }
    // check evaluation
    if(bottom_tracking.evaluation[0] <= 0.0 ||
        bottom_tracking.evaluation[1] <= 0.0 ||
        bottom_tracking.evaluation[2] <= 0.0 ||
        bottom_tracking.evaluation[3] <= 0.0)
    {
        return false;
    }
    if(bottom_tracking.velocity[3] > _max_error_velocity.value())
        return false;
    // check ranges
    if(bottom_tracking.range[0] <= _min_ground_distance.value() ||
        bottom_tracking.range[1] <= _min_ground_distance.value() ||
        bottom_tracking.range[2] <= _min_ground_distance.value() ||
        bottom_tracking.range[3] <= _min_ground_distance.value())
    {
        return false;
    }

    return true;
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See DVLBottomTrackingEvaluation.hpp for more detailed
// documentation about them.

bool DVLBottomTrackingEvaluation::configureHook()
{
    if (! DVLBottomTrackingEvaluationBase::configureHook())
        return false;
    return true;
}
bool DVLBottomTrackingEvaluation::startHook()
{
    if (! DVLBottomTrackingEvaluationBase::startHook())
        return false;

    return true;
}
void DVLBottomTrackingEvaluation::updateHook()
{
    DVLBottomTrackingEvaluationBase::updateHook();

    dvl_teledyne::BottomTracking bottom_tracking;
    while(_bottom_tracking_samples.read(bottom_tracking, false) == RTT::NewData)
    {
        base::samples::RigidBodyState velocity_sample;
        velocity_sample.time = bottom_tracking.time;
        if(evaluateBottomTrackingQuality(bottom_tracking))
        {
            velocity_sample.velocity << bottom_tracking.velocity[0], bottom_tracking.velocity[1], bottom_tracking.velocity[2];
            velocity_sample.cov_velocity = pow(_sigma.value(), 2.0) * Eigen::Matrix3d::Identity();
        }
        _velocity_samples.write(velocity_sample);
    }
}
void DVLBottomTrackingEvaluation::errorHook()
{
    DVLBottomTrackingEvaluationBase::errorHook();
}
void DVLBottomTrackingEvaluation::stopHook()
{
    DVLBottomTrackingEvaluationBase::stopHook();
}
void DVLBottomTrackingEvaluation::cleanupHook()
{
    DVLBottomTrackingEvaluationBase::cleanupHook();
}
