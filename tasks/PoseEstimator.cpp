/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "PoseEstimator.hpp"
#include <uwv_kalman_filters/PoseUKF.hpp>
#include <pose_estimation/StreamAlignmentVerifier.hpp>
#include <pose_estimation/GravitationalModel.hpp>
#include <base-logging/Logging.hpp>
#include "uwv_kalman_filtersTypes.hpp"
#include <ctime>

using namespace uwv_kalman_filters;

typedef PoseUKF::WState FilterState;

PoseEstimator::PoseEstimator(std::string const &name)
    : PoseEstimatorBase(name)
{
}

PoseEstimator::PoseEstimator(std::string const &name, RTT::ExecutionEngine *engine)
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
    measurement.mu << body_efforts_sample.linear, body_efforts_sample.angular;
    measurement.cov = cov_body_efforts;

    last_efforts_sample_time = ts;
    body_efforts_unknown = true;

    PoseUKF::State current_state;
    if (pose_filter->getCurrentState(current_state) && current_state.position.z() > dynamic_model_min_depth)
    {
        // inflate measurement uncertainty when close to the surface
        measurement.cov = cov_body_efforts_unknown;
    }

    try
    {
        // apply linear body effort measurement
        pose_filter->integrateMeasurement(measurement);
        body_efforts_unknown = false;
    }
    catch (const std::runtime_error &e)
    {
        LOG_ERROR_S << "Failed to integrate body effort measurements: " << e.what();
    }
}

void PoseEstimator::dvl_velocity_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &dvl_velocity_samples_sample)
{
    // receive sensor to body transformation
    Eigen::Affine3d dvlInIMU;
    if (!_dvl2body.get(ts, dvlInIMU))
    {
        LOG_ERROR_S << "skip, couldn't receive a valid dvl-in-body transformation sample!";
        new_state = MISSING_TRANSFORMATION;
        return;
    }
    dvlInIMU.translation() -= imu_in_body.translation();

    if (dvl_velocity_samples_sample.hasValidVelocity() && dvl_velocity_samples_sample.hasValidVelocityCovariance())
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
        catch (const std::runtime_error &e)
        {
            LOG_ERROR_S << "Failed to integrate DVL measurement: " << e.what();
        }
    }
}

void PoseEstimator::ground_distance_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &ground_distance_samples_sample)
{
    if (base::isNaN(ground_distance_samples_sample.position[2]))
        ground_distance = base::infinity<double>();
    else
        ground_distance = ground_distance_samples_sample.position[2];
}

void PoseEstimator::water_current_samplesTransformerCallback(const base::Time &ts, const dvl_teledyne::CellReadings &water_current_samples_sample)
{
    if (base::isNaN(ground_distance))
    {
        LOG_ERROR_S << "ground distance not initialized, not taking ADCP measurement";
        return;
    }

    // receive sensor to body transformation
    Eigen::Affine3d dvlInIMU;
    if (!_dvl2body.get(ts, dvlInIMU))
    {
        LOG_ERROR_S << "skip, couldn't receive a valid dvl-in-body (for ADCP measurement) transformation sample!";
        new_state = MISSING_TRANSFORMATION;
        return;
    }
    dvlInIMU.translation() -= imu_in_body.translation();

    // length of measurement vector should be variable based on height and return quality
    int num_cells = water_current_samples_sample.readings.size();
    if (!base::isInfinity(ground_distance))
        num_cells = std::min(num_cells, (int)floor(ground_distance / water_profiling_cell_size));
    double min_corr;
    double cell_weighting;
    base::Vector3d velocity;
    PoseUKF::WaterVelocityMeasurement measurement;

    for (int i = 0; i < num_cells; i++)
    {
        min_corr = *std::min_element(water_current_samples_sample.readings[i].correlation, water_current_samples_sample.readings[i].correlation + 4);

        if (min_corr > water_profiling_min_correlation && Eigen::Map<const Eigen::Vector3f>(water_current_samples_sample.readings[i].velocity).allFinite())
        {
            velocity << water_current_samples_sample.readings[i].velocity[0], water_current_samples_sample.readings[i].velocity[1], water_current_samples_sample.readings[i].velocity[2];
            velocity = dvlInIMU.rotation() * velocity;
            velocity -= pose_filter->getRotationRate().cross(dvlInIMU.translation());

            measurement.mu = velocity.head<2>();
            measurement.cov = (dvlInIMU.rotation() * cov_water_velocity * dvlInIMU.rotation().transpose()).topLeftCorner(2, 2);

            cell_weighting = (double(i) * water_profiling_cell_size + 0.5 * water_profiling_cell_size + water_profiling_first_cell_blank - dvlInIMU.translation().z()) /
                             ((double)water_current_samples_sample.readings.size() * water_profiling_cell_size + water_profiling_first_cell_blank - dvlInIMU.translation().z());

            try
            {
                pose_filter->integrateMeasurement(measurement, cell_weighting);
            }
            catch (const std::runtime_error &e)
            {
                LOG_ERROR_S << "Failed to integrate ADCP measurement: " << e.what();
            }
        }
    }
}

void PoseEstimator::imu_sensor_samplesTransformerCallback(const base::Time &ts, const ::base::samples::IMUSensors &imu_sensor_samples_sample)
{
    // prediction step
    predictionStep(ts);

    // apply new gyro measurement
    PoseUKF::RotationRate rotation_rate;
    rotation_rate.mu = imu_in_body.rotation() * imu_sensor_samples_sample.gyro;
    rotation_rate.cov = cov_angular_velocity;

    try
    {
        pose_filter->integrateMeasurement(rotation_rate);
    }
    catch (const std::runtime_error &e)
    {
        LOG_ERROR_S << "Failed to integrate angular velocity measurement: " << e.what();
    }

    // apply new acceleration measurement
    PoseUKF::Acceleration acceleration;
    acceleration.mu = imu_in_body.rotation() * imu_sensor_samples_sample.acc;
    acceleration.cov = cov_acceleration;

    try
    {
        pose_filter->integrateMeasurement(acceleration);
    }
    catch (const std::runtime_error &e)
    {
        LOG_ERROR_S << "Failed to integrate acceleration measurement: " << e.what();
    }

    // integrate zero effort measurement with a sigma of max effort in order to constrain the velocity
    if (body_efforts_unknown)
    {
        try
        {
            PoseUKF::BodyEffortsMeasurement measurement;
            measurement.mu = Eigen::Matrix<double, 6, 1>::Zero();
            measurement.cov = cov_body_efforts_unavailable;

            pose_filter->integrateMeasurement(measurement, true);
        }
        catch (const std::runtime_error &e)
        {
            LOG_ERROR_S << "Failed to integrate zero effort measurement: " << e.what();
        }
    }
    else if ((ts - last_efforts_sample_time).toSeconds() > (_body_efforts_period.value() * 3.0))
        body_efforts_unknown = true;

    // write current guess
    writeEstimatedState();
}

void PoseEstimator::altitude_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &pressure_sensor_samples_sample)
{
    // receive sensor to IMU transformation
    Eigen::Affine3d pressureSensorInIMU;
    if (!_pressure_sensor2body.get(ts, pressureSensorInIMU))
    {
        LOG_ERROR_S << "skip, couldn't receive a valid pressure-sensor-in-body transformation sample!";
        new_state = MISSING_TRANSFORMATION;
        return;
    }
    pressureSensorInIMU.translation() -= imu_in_body.translation();

    PoseUKF::State current_state;
    if (pose_filter->getCurrentState(current_state))
    {
        Eigen::Matrix<double, 1, 1> altitude;
        altitude << pressure_sensor_samples_sample.position.z() - (current_state.orientation * pressureSensorInIMU.translation()).z();

        // apply altitude measurement
        PoseUKF::Z_Position measurement;
        measurement.mu = altitude;
        measurement.cov = pressure_sensor_samples_sample.cov_position.bottomRightCorner(1, 1);

        try
        {
            pose_filter->integrateMeasurement(measurement);
        }
        catch (const std::runtime_error &e)
        {
            LOG_ERROR_S << "Failed to integrate altitude measurement: " << e.what();
        }
    }
}

void PoseEstimator::pressure_samplesTransformerCallback(const base::Time &ts, const base::samples::Pressure &pressure_samples_sample)
{
    // receive sensor to IMU transformation
    Eigen::Affine3d pressureSensorInIMU;
    if (!_pressure_sensor2body.get(ts, pressureSensorInIMU))
    {
        LOG_ERROR_S << "skip, couldn't receive a valid pressure-sensor-in-body transformation sample!";
        new_state = MISSING_TRANSFORMATION;
        return;
    }
    pressureSensorInIMU.translation() -= imu_in_body.translation();

    // apply pressure measurement
    PoseUKF::Pressure measurement;
    measurement.mu << pressure_samples_sample.pascal;
    measurement.cov << var_pressure;

    try
    {
        pose_filter->integrateMeasurement(measurement, pressureSensorInIMU.translation());
    }
    catch (const std::runtime_error &e)
    {
        LOG_ERROR_S << "Failed to integrate pressure measurement: " << e.what();
    }
}

void PoseEstimator::xy_position_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &xy_position_samples_sample)
{
    PoseUKF::State current_state;
    if (pose_filter->getCurrentState(current_state))
    {
        // apply xy measurement
        PoseUKF::XY_Position measurement;
        measurement.mu = nav_in_nwu_2d * (xy_position_samples_sample.position.head<2>() + (nwu_in_nav.rotation() * current_state.orientation * imu_in_body.translation()).head<2>());
        measurement.cov = nav_in_nwu_2d.linear() * xy_position_samples_sample.cov_position.topLeftCorner(2, 2) * nav_in_nwu_2d.linear().transpose();

        try
        {
            pose_filter->integrateMeasurement(measurement);
        }
        catch (const std::runtime_error &e)
        {
            LOG_ERROR_S << "Failed to integrate 2D position measurement: " << e.what();
        }
    }
}

void PoseEstimator::usbl_samplesTransformerCallback(const base::Time &ts, const base::samples::RigidBodyState &usbl_sample)
{
    Eigen::Affine3d usblModemInBody;
    Eigen::Affine3d usblBaseInNav;
    if (!_usbl_modem2body.get(ts, usblModemInBody))
    {
        LOG_ERROR_S << "skip, couldn't receive a valid usbl_modem-in-body transformation sample!";
        new_state = MISSING_TRANSFORMATION;
        return;
    }
    if (!_usbl_base2navigation.get(ts, usblBaseInNav))
    {
        LOG_ERROR_S << "skip, couldn't receive a valid usbl_base-in-nav transformation sample!";
        new_state = MISSING_TRANSFORMATION;
        return;
    }

    PoseUKF::State current_state;
    if (pose_filter->getCurrentState(current_state))
    {
        base::samples::RigidBodyState delayed_state;
        if (!this->findBestStateSampleInBuffer(usbl_sample.time, delayed_state))
            return;
        
        Eigen::Affine3d delayed_usblM_in_nav = delayed_state.getTransform() * usblModemInBody;
        Eigen::Affine3d delayed_usblM_in_base = usblBaseInNav.inverse() * delayed_usblM_in_nav;
        delayed_state.setTransform(delayed_usblM_in_base);
        base::samples::RigidBodyState pose_sample;
        pose_sample.position = nwu_in_nav * (Eigen::Vector3d(current_state.position) - current_state.orientation * imu_in_body.translation());
        pose_sample.orientation = nwu_in_nav.rotation() * current_state.orientation;
        Eigen::Affine3d current_usblM_in_nav = pose_sample.getTransform() * usblModemInBody;
        Eigen::Affine3d current_usblM_in_base = usblBaseInNav.inverse() * current_usblM_in_nav;
        pose_sample.setTransform(current_usblM_in_base);

        base::samples::RigidBodyState rbs_usbl_sample = usbl_sample; // usblMinUSBLBase
        // add moved distance between point where measurement was taken and current pose to measured position sample
        rbs_usbl_sample.position += delayed_state.position - pose_sample.position;
        rbs_usbl_sample.orientation = current_usblM_in_base.rotation();
        
        Eigen::Affine3d usblM_in_nav = usblBaseInNav * rbs_usbl_sample.getTransform();
        if (usblM_in_nav.translation().z() >= usblBaseInNav.translation().z())
            return; // simple outlier rejection, if modem over base, then there is a high probability for outliers
        Eigen::Affine3d body_in_nav = usblM_in_nav * usblModemInBody.inverse();
        //rbs_usbl_sample.setTransform(body_in_nav);
        
        std::cout << "USBL Body in Nav measurement: " << body_in_nav.translation() << std::endl;
        Eigen::Affine3d imu_in_nav = body_in_nav;
        imu_in_nav.translation() -= body_in_nav.rotation() * imu_in_body.translation();
        Eigen::Affine3d imu_in_nwu = nav_in_nwu * imu_in_nav;

        // TODO: Merge to XYZ Position ?
        PoseUKF::XY_Position xy_measurement;
        PoseUKF::Z_Position z_measurement;
        xy_measurement.mu = imu_in_nwu.translation().head<2>();
        // TODO: this is only a hack, it should be passed by the usbl driver
        // Eigen::Matrix2d xy_cov;
        // xy_cov(0, 0) = 0.2;
        // xy_cov(0, 1) = 0.;
        // xy_cov(1, 0) = 0.;
        // xy_cov(1, 1) = 0.2;
        xy_measurement.cov = usbl_sample.cov_position.topLeftCorner(2,2);
        z_measurement.mu = imu_in_nwu.translation().tail<1>();
        Eigen::Matrix<double, 1, 1> z_cov;
        z_cov(0, 0) = 0.2;
        z_measurement.cov = z_cov;

        try
        {
            pose_filter->integrateMeasurement(xy_measurement);
            //pose_filter->integrateMeasurement(z_measurement);
        }
        catch (const std::runtime_error &e)
        {
            LOG_ERROR_S << "Failed to integrate USBL Position measurement: " << e.what();
        }
    }
}

void PoseEstimator::gps_position_samplesTransformerCallback(const base::Time &ts, const base::samples::RigidBodyState &gps_position_samples_sample)
{
    // receive GPS to IMU transformation
    Eigen::Affine3d gpsInIMU;
    if (!_gps2body.get(ts, gpsInIMU))
    {
        LOG_ERROR_S << "skip, couldn't receive a valid gps-in-body transformation sample!";
        new_state = MISSING_TRANSFORMATION;
        return;
    }
    gpsInIMU.translation() -= imu_in_body.translation();

    PoseUKF::State current_state;
    if (pose_filter->getCurrentState(current_state))
    {
        PoseUKF::XY_Position measurement;
        measurement.mu = gps_position_samples_sample.position.head<2>() - (current_state.orientation * gpsInIMU.translation()).head<2>();
        measurement.cov = gps_position_samples_sample.cov_position.topLeftCorner(2, 2);

        try
        {
            pose_filter->integrateMeasurement(measurement);
        }
        catch (const std::runtime_error &e)
        {
            LOG_ERROR_S << "Failed to integrate 2D position measurement: " << e.what();
        }
    }
}

void PoseEstimator::gps_samplesTransformerCallback(const base::Time &ts, const gps_base::Solution &gps_samples_sample)
{
    // receive GPS to IMU transformation
    Eigen::Affine3d gpsInIMU;
    if (!_gps2body.get(ts, gpsInIMU))
    {
        LOG_ERROR_S << "skip, couldn't receive a valid gps-in-body transformation sample!";
        new_state = MISSING_TRANSFORMATION;
        return;
    }
    gpsInIMU.translation() -= imu_in_body.translation();

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
    catch (const std::runtime_error &e)
    {
        LOG_ERROR_S << "Failed to integrate GPS measurement: " << e.what();
    }
}

void PoseEstimator::apriltag_featuresTransformerCallback(const base::Time &ts, const apriltags::VisualFeaturePoints &visual_features_samples)
{
    // receive camera to body transformation
    Eigen::Affine3d cameraInBody;
    if (!_camera2body.get(ts, cameraInBody))
    {
        LOG_ERROR_S << "skip, couldn't receive a valid camera-in-body transformation sample!";
        new_state = MISSING_TRANSFORMATION;
        return;
    }
    Eigen::Affine3d cameraInIMU = cameraInBody;
    cameraInIMU.translation() += imu_in_body.translation();

    for (unsigned i = 0; i < visual_features_samples.feature_points.size(); i++)
    {
        const apriltags::VisualFeaturePoint &feature_points = visual_features_samples.feature_points[i];
        std::map<std::string, VisualMarker>::const_iterator landmark = known_landmarks.find(feature_points.identifier);
        if (landmark == known_landmarks.end())
        {
            LOG_WARN_S << "Landmark with ID " << feature_points.identifier << " is unknown. Measurements will be skipped.";
            continue;
        }
        else if (feature_points.points.size() > landmark->second.feature_positions.size())
        {
            LOG_ERROR_S << "More then " << landmark->second.feature_positions.size() << " visual features are not supported. Measurements will be skipped.";
            continue;
        }

        std::vector<PoseUKF::VisualFeatureMeasurement> measurements(feature_points.points.size());
        for (unsigned j = 0; j < feature_points.points.size(); j++)
        {
            measurements[j].mu = feature_points.points[j];
            measurements[j].cov = cov_visual_feature;
        }

        try
        {
            pose_filter->integrateMeasurement(measurements, landmark->second.feature_positions,
                                              landmark->second.marker_pose, landmark->second.cov_marker_pose,
                                              camera_config, cameraInIMU);
        }
        catch (const std::runtime_error &e)
        {
            LOG_ERROR_S << "Failed to integrate visual measurement: " << e.what();
        }
    }
}

void PoseEstimator::apriltags_marker_poses_stampedTransformerCallback(const base::Time &ts, const apriltags::MarkerPosesStamped &marker_poses_stamped_samples)
{
    // receive camera to body transformation
    Eigen::Affine3d cameraInBody;
    if (!_camera2body.get(ts, cameraInBody))
    {
        LOG_ERROR_S << "skip, couldn't receive a valid camera-in-body transformation sample!";
        new_state = MISSING_TRANSFORMATION;
        return;
    }

    Eigen::Affine3d imu_in_nwu_final;
    for (size_t i = 0; i < marker_poses_stamped_samples.marker_poses.size(); i++)
    {
        // TODO check if sourceFrame is correct to receive the tag id
        std::string erase = "_frame";
        std::string marker_id = marker_poses_stamped_samples.marker_poses[i].sourceFrame;
        marker_id = marker_id.erase(marker_id.find(erase), erase.length());
        // PoseUKF::State current_state;
        // if (pose_filter->getCurrentState(current_state))
        // {
        //     Eigen::Affine3d marker_in_body = cameraInBody * marker_pose.getTransform();
        //     base::samples::RigidBodyState rbs_body_in_nav;
        //     rbs_body_in_nav.position = nwu_in_nav * (Eigen::Vector3d(current_state.position) - current_state.orientation * imu_in_body.translation());
        //     rbs_body_in_nav.orientation = nwu_in_nav.rotation() * current_state.orientation;
        //     Eigen::Affine3d marker_in_nav_print = rbs_body_in_nav.getTransform() * marker_in_body;
        //     std::cout << "Marker: " << marker_id << " Position: " << marker_in_nav_print.translation() << " Orientation: " << marker_in_nav_print.rotation() << std::endl;
        // }

        std::map<std::string, VisualMarker>::const_iterator landmark = known_landmarks.find(marker_id);
        if (landmark == known_landmarks.end())
        {
            std::cout << "Marker with ID " << marker_id << " is unknown. Measurements will be skipped." << std::endl;
            continue;
        }
        if (marker_poses_stamped_samples.marker_poses[i].position[2] > 2.) continue; //temporary
        Eigen::Affine3d cameraInIMU = imu_in_body.inverse() * cameraInBody;
        Eigen::Affine3d markerInImu = cameraInIMU * marker_poses_stamped_samples.marker_poses[i].getTransform();

        Eigen::Affine3d ImuInNWU_measurement = landmark->second.marker_pose * markerInImu.inverse(); // landmark->second.marker_pose * marker_pose.getTransform().inverse() * cameraInBody.inverse() * imu_in_body;

        // std::cout << "Body In Nav: " << ((nav_in_nwu.inverse() * ImuInNWU_measurement) * imu_in_body.inverse()).translation() << std::endl;

        // TODO: Average pose estimate based on marker pose reading
        if (i == 0)
        {
            imu_in_nwu_final = ImuInNWU_measurement;
        }
        else
        {
            std::cout << "Before Averaging: " << imu_in_nwu_final.translation() << std::endl;
            imu_in_nwu_final.translation() = (imu_in_nwu_final.translation() + ImuInNWU_measurement.translation()) / 2;
            std::cout << "After Averaging: " << imu_in_nwu_final.translation() << std::endl;
            Eigen::Quaterniond q_1(imu_in_nwu_final.rotation());
            Eigen::Quaterniond q_2(ImuInNWU_measurement.rotation());
            imu_in_nwu_final.linear() = q_1.slerp(0.5, q_2).toRotationMatrix();
        }
    }
    std::cout << "Final t: " << imu_in_nwu_final.translation() << std::endl;
    std::cout << "Final r: " << imu_in_nwu_final.rotation() << std::endl;
    // Reset Pose Filter based on marker measurement
    // TODO: This whole resetting thingy should be handled either by internal ukf or by something like a RLS Filter as described in https://www.researchgate.net/publication/330591847_Long-Duration_Autonomy_for_Small_Rotorcraft_UAS_Including_Recharging
    pose_filter->resetFilterWithExternalPose(imu_in_nwu_final);
}

void PoseEstimator::integrateDelayedPositionSamples(const base::Time &ts)
{
    base::samples::RigidBodyState rbs;
    while (_delayed_xy_position_samples.read(rbs) == RTT::NewData)
    {
        delayed_position_samples.push_back(rbs);
    }

    while (!delayed_position_samples.empty() && delayed_position_samples.front().time < ts)
    {
        // NOTE Currently the displacement from the delayed sensor position to the IMU frame is disregarded.
        // apply xy measurement
        PoseUKF::XY_Position measurement;
        measurement.mu = nav_in_nwu_2d * delayed_position_samples.front().position.head<2>();
        measurement.cov = nav_in_nwu_2d.linear() * delayed_position_samples.front().cov_position.topLeftCorner(2, 2) * nav_in_nwu_2d.linear().transpose();

        try
        {
            double delay = (ts - delayed_position_samples.front().time).toSeconds();
            if (!pose_filter->integrateDelayedMeasurement(measurement, delay))
            {
                LOG_ERROR_S << "Failed to integrate delayed 2D position measurement: Maximum delay exceeded (" << delay << " seconds).";
            }
        }
        catch (const std::runtime_error &e)
        {
            LOG_ERROR_S << "Failed to integrate delayed 2D position measurement: " << e.what();
        }

        delayed_position_samples.pop_front();
    }
}

void PoseEstimator::predictionStep(const base::Time &sample_time)
{
    try
    {
        pose_filter->predictionStepFromSampleTime(sample_time);
    }
    catch (const std::runtime_error &e)
    {
        LOG_ERROR_S << "Failed to execute prediction step: " << e.what();
        LOG_ERROR_S << "Skipping prediction step.";
    }

    integrateDelayedPositionSamples(sample_time);
}

void PoseEstimator::writeEstimatedState()
{
    // write estimated body state
    PoseUKF::State current_state;
    PoseUKF::Covariance state_cov;
    base::Time current_sample_time = base::Time::fromMicroseconds(pose_filter->getLastMeasurementTime());
    if (current_sample_time > last_sample_time && pose_filter->getCurrentState(current_state, state_cov))
    {
        /* The pose filter State is in body-aligned IMU in NWU frame, which means that it is translated to IMU, but rotation is body-aligned (e.g. x-forward looking) */
        /* Transform filter state from body-aligned IMU in NWU aligned navigation frame to
         * body in navigation frame */
        base::samples::RigidBodyState pose_sample;
        pose_sample.position = nwu_in_nav * (Eigen::Vector3d(current_state.position) - current_state.orientation * imu_in_body.translation());
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

        this->addStateToBuffer(pose_sample);

        SecondaryStates secondary_states;
        secondary_states.acceleration = nwu_in_nav.rotation() * current_state.acceleration;
        secondary_states.cov_acceleration = nwu_in_nav.linear() * MTK::subblock(state_cov, &FilterState::acceleration) * nwu_in_nav.linear().transpose();
        secondary_states.bias_gyro = current_state.bias_gyro;
        secondary_states.cov_bias_gyro = MTK::subblock(state_cov, &FilterState::bias_gyro);
        secondary_states.bias_acc = current_state.bias_acc;
        secondary_states.cov_bias_acc = MTK::subblock(state_cov, &FilterState::bias_acc);
        secondary_states.gravity = current_state.gravity(0);
        secondary_states.var_gravity = MTK::subblock(state_cov, &FilterState::gravity)(0);
        secondary_states.inertia = current_state.inertia.cwiseAbs();
        secondary_states.cov_inertia_diag = MTK::subblock(state_cov, &FilterState::inertia).diagonal();
        secondary_states.lin_damping = current_state.lin_damping.cwiseAbs();
        secondary_states.cov_lin_damping_diag = MTK::subblock(state_cov, &FilterState::lin_damping).diagonal();
        secondary_states.quad_damping = current_state.quad_damping.cwiseAbs();
        secondary_states.cov_quad_damping_diag = MTK::subblock(state_cov, &FilterState::quad_damping).diagonal();
        secondary_states.water_velocity = current_state.water_velocity;
        secondary_states.cov_water_velocity = MTK::subblock(state_cov, &FilterState::water_velocity);
        secondary_states.water_velocity_below = current_state.water_velocity_below;
        secondary_states.cov_water_velocity_below = MTK::subblock(state_cov, &FilterState::water_velocity_below);
        secondary_states.bias_adcp = current_state.bias_adcp;
        secondary_states.cov_bias_adcp = MTK::subblock(state_cov, &FilterState::bias_adcp);
        secondary_states.water_density = current_state.water_density(0);
        secondary_states.var_water_density = MTK::subblock(state_cov, &FilterState::water_density)(0);
        secondary_states.time = current_sample_time;
        _secondary_states.write(secondary_states);

        last_sample_time = current_sample_time;
        this->manageBufferSize();
    }
}

bool PoseEstimator::initializeFilter(const base::samples::RigidBodyState &initial_rbs,
                                     const PoseUKFConfig &filter_config,
                                     const uwv_dynamic_model::UWVParameters &model_parameters,
                                     const Eigen::Affine3d &imu_in_body,
                                     const Eigen::Affine3d &nav_in_nwu)
{
    if (!initial_rbs.hasValidPosition() || !initial_rbs.hasValidPositionCovariance() || !initial_rbs.hasValidOrientation() || !initial_rbs.hasValidOrientationCovariance())
    {
        LOG_ERROR_S << "Undefined values in initial pose!";
        return false;
    }

    if (model_parameters.damping_matrices.size() < 2)
    {
        LOG_ERROR_S << "The damping matrices must have at least two elements!";
        return false;
    }

    Eigen::Vector3d position_nwu = nav_in_nwu * (initial_rbs.position + initial_rbs.orientation * imu_in_body.translation());
    Eigen::Quaterniond orientation_nwu = Eigen::Quaterniond(nav_in_nwu.rotation() * initial_rbs.orientation);
    Eigen::Matrix3d cov_position_nwu = nav_in_nwu.linear() * initial_rbs.cov_position * nav_in_nwu.linear().transpose();
    Eigen::Matrix3d cov_orientation_nwu = nav_in_nwu.linear() * initial_rbs.cov_orientation * nav_in_nwu.linear().transpose();

    pose_filter.reset(new PoseUKF(position_nwu, cov_position_nwu,
                                  orientation_nwu, cov_orientation_nwu,
                                  filter_config, model_parameters,
                                  imu_in_body));
    return true;
}

bool PoseEstimator::setProcessNoise(const PoseUKFConfig &filter_config, double imu_delta_t, const Eigen::Affine3d &imu_in_body)
{
    pose_filter->setProcessNoiseFromConfig(filter_config, imu_delta_t, Eigen::Quaterniond(imu_in_body.rotation()));
    PoseUKF::Covariance process_noise_cov = pose_filter->getProcessNoiseCovariance();

    LOG_DEBUG_S << "(sigma/delta_t)^2 position:\n"
                << MTK::subblock(process_noise_cov, &FilterState::position);
    LOG_DEBUG_S << "(sigma/delta_t)^2 velocity:\n"
                << MTK::subblock(process_noise_cov, &FilterState::velocity);
    LOG_DEBUG_S << "(sigma/delta_t)^2 acceleration:\n"
                << MTK::subblock(process_noise_cov, &FilterState::acceleration);

    return true;
}

void PoseEstimator::registerKnownLandmarks(const VisualLandmarkConfiguration &config, const Eigen::Affine3d &nav_in_nwu)
{
    const std::vector<VisualLandmark> &landmarks = config.landmarks;
    const std::vector<base::Vector3d> &unit_feature_positions = config.unit_feature_positions;
    for (unsigned i = 0; i < landmarks.size(); i++)
    {
        Eigen::Affine3d marker_in_nav(Eigen::AngleAxisd(landmarks[i].marker_euler_orientation.z(), Eigen::Vector3d::UnitZ()) *
                                      Eigen::AngleAxisd(landmarks[i].marker_euler_orientation.y(), Eigen::Vector3d::UnitY()) *
                                      Eigen::AngleAxisd(landmarks[i].marker_euler_orientation.x(), Eigen::Vector3d::UnitX()));
        marker_in_nav.translation() = landmarks[i].marker_position;

        VisualMarker landmark;
        landmark.marker_pose = nav_in_nwu * marker_in_nav;
        landmark.cov_marker_pose = Eigen::Matrix<double, 6, 6>::Zero();
        landmark.cov_marker_pose.topLeftCorner<3, 3>() = nav_in_nwu.rotation() * landmarks[i].marker_pose_std.head<3>().cwiseAbs2().asDiagonal() * nav_in_nwu.rotation().transpose();
        landmark.cov_marker_pose.bottomRightCorner<3, 3>() = nav_in_nwu.rotation() * landmarks[i].marker_pose_std.tail<3>().cwiseAbs2().asDiagonal() * nav_in_nwu.rotation().transpose();
        landmark.feature_positions.resize(unit_feature_positions.size());
        for (unsigned j = 0; j < unit_feature_positions.size(); j++)
        {
            landmark.feature_positions[j] = unit_feature_positions[j] * landmarks[i].marker_size * 0.5;
        }
        known_landmarks[landmarks[i].marker_id] = landmark;
    }
}

void PoseEstimator::addStateToBuffer(const base::samples::RigidBodyState &state)
{
    state_buffer_.push_back(state);
}

void PoseEstimator::manageBufferSize()
{
    while (!state_buffer_.empty())
    {
        auto diff = abs((last_sample_time - state_buffer_.front().time).toMilliseconds());
        if (diff > state_buffer_duration_)
        {
            this->removeStateSampleFromBufferFront();
        }
        else
        {
            return;
        }
    }
}

void PoseEstimator::removeStateSampleFromBufferFront()
{
    state_buffer_.pop_front();
}

bool PoseEstimator::findBestStateSampleInBuffer(const base::Time &time_sample, base::samples::RigidBodyState &output_state)
{
    // TODO: are there any other cases in which to return false ? E.g. delayed sensor measurement is to far in the past ?
    if (state_buffer_.empty())
    {
        return false;
    }

    unsigned int last_time_diff = 2 * state_buffer_duration_; // ensure that initial time diff is bigger than max time_diff between now and oldest sample, kind of a hack
    base::samples::RigidBodyState last_sample;
    // loop through state buffer starting at the last sample. This should be faster given that the delayed sensor measurement arrives alot faster than the state buffer is buffering samples. This will be slower if time difference between sample time and current time > 0.5 * state_buffer_duration
    for (auto rit = state_buffer_.rbegin(); rit != state_buffer_.rend(); ++rit)
    {
        auto sample = *rit;
        auto diff = abs((time_sample - sample.time).toMilliseconds());
        if (diff < last_time_diff)
        {
            last_time_diff = diff;
            last_sample = sample;
        }
        else
        {
            if (last_time_diff > max_time_diff_to_state_)
                return false;
            output_state = last_sample;
            return true;
        }
    }
    return false;
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See PoseEstimator.hpp for more detailed
// documentation about them.

bool PoseEstimator::configureHook()
{
    if (!PoseEstimatorBase::configureHook())
        return false;
    

    // get IMU to body transformation
    if (!_imu2body.get(base::Time(), imu_in_body))
    {
        LOG_ERROR_S << "Failed to get IMU pose in body frame. Note that this has to be a static transformation!";
        return false;
    }

    // get navigation in NWU aligned frame
    if (!_navigation2navigation_nwu.get(base::Time(), nav_in_nwu))
    {
        LOG_ERROR_S << "Failed to get navigation in NWU frame. Note that this has to be a static transformation!";
        return false;
    }
    base::Vector3d nav_in_nwu_euler = base::getEuler(base::Orientation(nav_in_nwu.linear()));
    if (nav_in_nwu_euler.y() != 0. || nav_in_nwu_euler.z() != 0.)
    {
        LOG_ERROR_S << "The navigation frame can only be rotated with respect to the z axis in the NWU frame!";
        return false;
    }
    nwu_in_nav = nav_in_nwu.inverse();
    nav_in_nwu_2d = Eigen::Affine2d(Eigen::Rotation2Dd(nav_in_nwu_euler.x()));
    nav_in_nwu_2d.translation() = nwu_in_nav.translation().head<2>();

    // compute measurement covariances
    double sqrt_delta_t = sqrt(_imu_sensor_samples_period.value());
    Eigen::Vector3d rotation_rate_std = (1. / sqrt_delta_t) * _filter_config.value().rotation_rate.randomwalk;
    Eigen::Vector3d acceleration_std = (1. / sqrt_delta_t) * _filter_config.value().acceleration.randomwalk;
    cov_angular_velocity = imu_in_body.rotation() * rotation_rate_std.cwiseAbs2().asDiagonal() * imu_in_body.rotation().transpose();
    cov_acceleration = imu_in_body.rotation() * acceleration_std.cwiseAbs2().asDiagonal() * imu_in_body.rotation().transpose();
    cov_body_efforts = (1. / _body_efforts_period.value()) * _filter_config.value().model_noise_parameters.body_efforts_std.cwiseAbs2().asDiagonal();
    cov_body_efforts_unknown = (1. / _body_efforts_period.value()) * (_filter_config.value().max_effort.cwiseAbs2()).asDiagonal();
    cov_body_efforts_unavailable = (1. / _imu_sensor_samples_period.value()) * (_filter_config.value().max_effort.cwiseAbs2()).asDiagonal();
    cov_water_velocity = (1. / _water_current_samples_period.value()) * (_filter_config.value().water_velocity.measurement_std.cwiseAbs2()).asDiagonal();
    cov_visual_feature = (1. / _apriltag_features_period.value()) * _filter_config.value().visual_landmarks.feature_std.cwiseAbs2().asDiagonal();
    var_pressure = (1. / _pressure_samples_period.value()) * pow(_filter_config.value().hydrostatics.pressure_std, 2.0);

    dynamic_model_min_depth = _filter_config.value().dynamic_model_min_depth;

    water_profiling_min_correlation = _filter_config.value().water_velocity.minimum_correlation;
    water_profiling_cell_size = _filter_config.value().water_velocity.cell_size;
    water_profiling_first_cell_blank = _filter_config.value().water_velocity.first_cell_blank;

    camera_config = _filter_config.value().visual_landmarks.camera_config;

    // register known landmarks
    registerKnownLandmarks(_filter_config.value().visual_landmarks, nav_in_nwu);

    state_buffer_duration_ = _state_buffer_duration.value();
    state_buffer_.resize(state_buffer_duration);
    max_time_diff_to_state_ = _max_time_diff_to_state.value();

    return true;
}
bool PoseEstimator::startHook()
{
    if (!PoseEstimatorBase::startHook())
        return false;

    // initialize filter
    if (!initializeFilter(_initial_state.value(), _filter_config.value(), _model_parameters.value(), imu_in_body, nav_in_nwu))
        return false;

    // set process noise
    if (!setProcessNoise(_filter_config.value(), _imu_sensor_samples_period.value(), imu_in_body))
        return false;

    pose_filter->setMaxTimeDelta(_max_time_delta.get());

    // setup delayed state buffer
    if (_max_delay_pos_measurement.rvalue() > 0)
    {
        pose_filter->setupDelayedStateBuffer(_max_delay_pos_measurement.rvalue());
    }

    // setup stream alignment verifier
    verifier.reset(new pose_estimation::StreamAlignmentVerifier());
    verifier->setVerificationInterval(20.0);
    verifier->setDropRateWarningThreshold(0.5);
    verifier->setDropRateCriticalThreshold(1.0);
    streams_with_alignment_failures = 0;
    streams_with_critical_alignment_failures = 0;

    // reset state machine related members
    last_sample_time = base::Time();
    last_efforts_sample_time = base::Time();
    last_state = PRE_OPERATIONAL;
    new_state = RUNNING;
    ground_distance = base::NaN<double>();
    body_efforts_unknown = false;

    return true;
}
void PoseEstimator::updateHook()
{
    new_state = RUNNING;
    PoseEstimatorBase::updateHook();

    // check stream alignment status
    verifier->verifyStreamAlignerStatus(_transformer.getStreamAlignerStatus(), streams_with_alignment_failures, streams_with_critical_alignment_failures);
    if (streams_with_alignment_failures > 0)
        new_state = TRANSFORMATION_ALIGNMENT_FAILURES;
    if (streams_with_critical_alignment_failures > 0)
        exception(CRITICAL_ALIGNMENT_FAILURE);

    // write task state if it has changed
    if (last_state != new_state)
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
