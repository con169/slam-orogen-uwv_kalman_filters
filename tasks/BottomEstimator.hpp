/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef UWV_KALMAN_FILTERS_BOTTOMESTIMATOR_TASK_HPP
#define UWV_KALMAN_FILTERS_BOTTOMESTIMATOR_TASK_HPP

#include "uwv_kalman_filters/BottomEstimatorBase.hpp"
#include <uwv_kalman_filters/BottomUKF.hpp>
#include <boost/shared_ptr.hpp>
#include <pose_estimation/StreamAlignmentVerifier.hpp>

namespace uwv_kalman_filters{

    /*! \class BottomEstimator
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * 
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','uwv_kalman_filters::BottomEstimator')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument.
     */
    class BottomEstimator : public BottomEstimatorBase
    {
	friend class BottomEstimatorBase;
    protected:
        boost::shared_ptr<uwv_kalman_filters::BottomUKF> bottom_filter;
        boost::shared_ptr<pose_estimation::StreamAlignmentVerifier> verifier;
        BottomEstimatorParameters config;
        double range_measurement_var;
        base::Time normal_measurement_time;
        NormalType normal_measurement;
        Eigen::Matrix2d normal_measurement_cov;
        Eigen::Affine3d transducer_in_body[4];
        Eigen::Affine3d current_body_pose;
        Eigen::Affine3d last_body_pose;
        bool valid_body_pose;
        base::Time last_sample_time;
        unsigned streams_with_alignment_failures;
        unsigned streams_with_critical_alignment_failures;
        States last_state;
        States new_state;

        virtual void dvl_bottom_trackingTransformerCallback(const base::Time &ts, const ::dvl_teledyne::BottomTracking &dvl_bottom_tracking_sample);

        virtual void pose_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &pose_samples_sample);

        void predictionStep(const base::Time& sample_time);

    public:
        /** TaskContext constructor for BottomEstimator
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        BottomEstimator(std::string const& name = "uwv_kalman_filters::BottomEstimator");

        /** Default deconstructor of BottomEstimator
         */
	~BottomEstimator();

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

