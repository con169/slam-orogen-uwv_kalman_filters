/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef UWV_KALMAN_FILTERS_POSEESTIMATOR_TASK_HPP
#define UWV_KALMAN_FILTERS_POSEESTIMATOR_TASK_HPP

#include "uwv_kalman_filters/PoseEstimatorBase.hpp"
#include <boost/shared_ptr.hpp>
#include <base/commands/LinearAngular6DCommand.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <base/samples/IMUSensors.hpp>
#include <uwv_kalman_filters/PoseUKFConfig.hpp>
#include <gps_base/BaseTypes.hpp>
#include <dvl_teledyne/PD0Messages.hpp>

namespace pose_estimation
{
    class StreamAlignmentVerifier;
};

namespace uwv_kalman_filters{

    class PoseUKF;

    /*! \class PoseEstimator
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * 
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','uwv_kalman_filters::PoseEstimator')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument.
     */
    class PoseEstimator : public PoseEstimatorBase
    {
	friend class PoseEstimatorBase;
    protected:
        boost::shared_ptr<uwv_kalman_filters::PoseUKF> pose_filter;
        boost::shared_ptr<pose_estimation::StreamAlignmentVerifier> verifier;
        Eigen::Affine3d imu_in_body;
        Eigen::Affine3d nav_in_nwu;
        Eigen::Affine3d nwu_in_nav;
        Eigen::Affine2d nav_in_nwu_2d;
        Eigen::Matrix3d cov_angular_velocity;
        Eigen::Matrix3d cov_acceleration;
        Eigen::Matrix3d cov_body_efforts;
        Eigen::Matrix3d cov_body_efforts_unknown;
        Eigen::Matrix3d cov_velocity_unknown;
        double dynamic_model_min_depth;
        double water_profiling_min_correlation;
        double water_profiling_cell_size;
        double water_profiling_first_cell_blank;
        unsigned streams_with_alignment_failures;
        unsigned streams_with_critical_alignment_failures;
        base::Time last_sample_time;
        States last_state;
        States new_state;
        double ground_distance;

        virtual void body_effortsTransformerCallback(const base::Time &ts, const ::base::commands::LinearAngular6DCommand &body_efforts_sample);

        virtual void dvl_velocity_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &dvl_velocity_samples_sample);

        virtual void ground_distance_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &ground_distance_samples_sample);

        virtual void water_current_samplesTransformerCallback(const base::Time &ts, const dvl_teledyne::CellReadings &water_current_samples_sample);

        virtual void imu_sensor_samplesTransformerCallback(const base::Time &ts, const ::base::samples::IMUSensors &imu_sensor_samples_sample);

        virtual void pressure_sensor_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &pressure_sensor_samples_sample);

        virtual void xy_position_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &xy_position_samples_sample);

        virtual void gps_position_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &gps_position_samples_sample);

        virtual void gps_samplesTransformerCallback(const base::Time &ts, const ::gps_base::Solution &gps_samples_sample);

        void predictionStep(const base::Time& sample_time);

        bool initializeFilter(const base::samples::RigidBodyState& initial_rbs, const PoseUKFConfig& filter_config,
                              const uwv_dynamic_model::UWVParameters& model_parameters, const Eigen::Affine3d& imu_in_body,
                              const Eigen::Affine3d& nav_in_nwu);

        bool setProcessNoise(const PoseUKFConfig& filter_config, double imu_delta_t);

    public:
        /** TaskContext constructor for PoseEstimator
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        PoseEstimator(std::string const& name = "uwv_kalman_filters::PoseEstimator");

        /** TaskContext constructor for PoseEstimator
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices.
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task.
         * 
         */
        PoseEstimator(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of PoseEstimator
         */
	~PoseEstimator();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states.
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();
    };
}

#endif

