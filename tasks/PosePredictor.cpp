/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "PosePredictor.hpp"
#include <mtk/types/SOn.hpp>

using namespace uwv_kalman_filters;

PosePredictor::PosePredictor(std::string const& name, TaskCore::TaskState initial_state)
    : PosePredictorBase(name, initial_state)
{
}

PosePredictor::PosePredictor(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : PosePredictorBase(name, engine, initial_state)
{
}

PosePredictor::~PosePredictor()
{
}

void PosePredictor::removeOldVelocitySamples(const base::Time& ts)
{
    while(!velocity_samples.empty() && velocity_samples.front().time <= ts)
    {
        velocity_samples.pop_front();
    }
}

void PosePredictor::predictPose(const base::samples::RigidBodyState& delayed_pose)
{
    prediced_pose.sourceFrame = delayed_pose.sourceFrame;
    prediced_pose.targetFrame = delayed_pose.targetFrame;
    prediced_pose.position = delayed_pose.position;
    prediced_pose.cov_position = delayed_pose.cov_position;
    prediced_pose.orientation = delayed_pose.orientation;
    prediced_pose.cov_orientation = delayed_pose.cov_orientation;
    prediced_pose.time = delayed_pose.time;

    // apply deltas
    for(std::list<base::samples::RigidBodyState>::const_iterator velocity_it = velocity_samples.begin();
        velocity_it != velocity_samples.end(); velocity_it++)
    {
        applyDelta(*velocity_it);
    }
}

void PosePredictor::applyDelta(const base::samples::RigidBodyState& velocity_sample, double min_delta)
{
    if(prediced_pose.time.isNull())
        return;

    double delta_t = (velocity_sample.time - prediced_pose.time).toSeconds();

    // avoid to small or negative time deltas
    if(delta_t < min_delta)
        return;

    // integrate delta
    prediced_pose.position += prediced_pose.orientation * velocity_sample.velocity * delta_t;
    MTK::SO3<double> orientation = MTK::SO3<double>(prediced_pose.orientation);
    orientation.boxplus(prediced_pose.orientation * velocity_sample.angular_velocity, delta_t);
    prediced_pose.orientation = orientation.normalized();
    prediced_pose.velocity = velocity_sample.velocity;
    prediced_pose.time = velocity_sample.time;
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See PosePredictor.hpp for more detailed
// documentation about them.

bool PosePredictor::configureHook()
{
    if (! PosePredictorBase::configureHook())
        return false;
    return true;
}
bool PosePredictor::startHook()
{
    if (! PosePredictorBase::startHook())
        return false;

    // invalidate predicted pose
    prediced_pose.invalidate();
    prediced_pose.time = base::Time();

    return true;
}
void PosePredictor::updateHook()
{
    PosePredictorBase::updateHook();

    // read in all velocity samples
    base::samples::RigidBodyState velocity_sample;
    while(_velocity_samples.read(velocity_sample) == RTT::NewData)
    {
        // apply new delta
        applyDelta(velocity_sample);

        velocity_samples.push_back(velocity_sample);
    }

    base::samples::RigidBodyState delayed_pose_sample;
    if(_delayed_pose_samples.read(delayed_pose_sample) == RTT::NewData)
    {
        // remove old velocity samples
        removeOldVelocitySamples(delayed_pose_sample.time);

        // apply velocity deltas
        predictPose(delayed_pose_sample);
    }

    if(prediced_pose.hasValidPosition())
    {
        // write out prediced pose
        _pose_samples.write(prediced_pose);
    }
}
void PosePredictor::errorHook()
{
    PosePredictorBase::errorHook();
}
void PosePredictor::stopHook()
{
    PosePredictorBase::stopHook();
}
void PosePredictor::cleanupHook()
{
    PosePredictorBase::cleanupHook();
}
