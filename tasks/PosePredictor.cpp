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

void PosePredictor::predictPose(const base::samples::RigidBodyState& delayed_pose,
                                base::samples::RigidBodyState& prediced_pose, double min_delta)
{
    prediced_pose.sourceFrame = delayed_pose.sourceFrame;
    prediced_pose.targetFrame = delayed_pose.targetFrame;
    
    // return current pose in case no newer velocity samples are available
    if(velocity_samples.empty())
    {
        prediced_pose.time = delayed_pose.time;
        prediced_pose.position = delayed_pose.position;
        prediced_pose.orientation = delayed_pose.orientation;
        return;
    }

    Eigen::Vector3d position = delayed_pose.position;
    MTK::SO3<double> orientation = MTK::SO3<double>(delayed_pose.orientation);

    base::Time last_ts = delayed_pose.time;
    for(std::list<base::samples::RigidBodyState>::const_iterator velocity_it = velocity_samples.begin();
        velocity_it != velocity_samples.end(); velocity_it++)
    {
        double delta_t = (velocity_it->time - last_ts).toSeconds();

        // avoid to small or negative time deltas
        if(delta_t < min_delta)
            continue;

        // integrate delta
        position += orientation * velocity_it->velocity * delta_t;
        orientation.boxplus(velocity_it->angular_velocity, delta_t);

        last_ts = velocity_it->time;
    }

    prediced_pose.time = velocity_samples.back().time;
    prediced_pose.position = position;
    prediced_pose.orientation = orientation.normalized();
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
    return true;
}
void PosePredictor::updateHook()
{
    PosePredictorBase::updateHook();

    // read in all velocity samples
    base::samples::RigidBodyState velocity_sample;
    while(_velocity_samples.read(velocity_sample) == RTT::NewData)
    {
        velocity_samples.push_back(velocity_sample);
    }

    base::samples::RigidBodyState delayed_pose_sample;
    if(_delayed_pose_samples.read(delayed_pose_sample) == RTT::NewData)
    {
        // remove old velocity samples
        removeOldVelocitySamples(delayed_pose_sample.time);

        // apply velocity deltas
        base::samples::RigidBodyState prediced_pose;
        predictPose(delayed_pose_sample, prediced_pose);

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
