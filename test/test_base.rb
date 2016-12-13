require 'minitest/spec'
require 'orocos/test/component'
require 'minitest/autorun'
require "transformer/runtime"

describe 'uwv_kalmam_filters::VelocityProvider configuration' do
    include Orocos::Test::Component
    start 'velocity_provider', 'uwv_kalman_filters::VelocityProvider' => 'velocity_provider'
    reader 'velocity_provider', 'velocity_samples', attr_name: 'velocity_samples'
    writer 'velocity_provider', 'body_efforts', attr_name: 'body_efforts'
    writer 'velocity_provider', 'dvl_velocity_samples', attr_name: 'dvl_velocity_samples'
    writer 'velocity_provider', 'pressure_sensor_samples', attr_name: 'pressure_sensor_samples'
    writer 'velocity_provider', 'imu_sensor_samples', attr_name: 'imu_sensor_samples'

    def matrix3_eye
	m = {:data => [1,0,0,0,1,0,0,0,1]}
	m
    end

    def matrix3_zero
	m = {:data => [0]*9}
	m
    end

    def matrix3_nan
	m = {:data => [NaN]*9}
	m
    end

    def zero_command
	sample = Types::Base::Commands.LinearAngular6DCommand.new
	sample.time = Time.now
	sample.linear = Types::Base::Vector3d.Zero
	sample.angular = Types::Base::Vector3d.Zero
	sample
    end

    def dvl_data(vel)
	sample = Types::Base::Samples.RigidBodyState.new
	sample.time = Time.now
	sample.velocity = vel
	sample.cov_velocity = matrix3_eye
	sample
    end

    def imu_data(ang_vel)
	sample = Types::Base::Samples.IMUSensors.new
	sample.time = Time.now
	sample.gyro = ang_vel
	sample.acc = Types::Base::Vector3d.Zero
	sample.mag = Types::Base::Vector3d.Zero
	sample
    end

    def depth_data(z)
	sample = Types::Base::Samples.RigidBodyState.new
	sample.time = Time.now
	sample.position[0] = 0
	sample.position[1] = 0
	sample.position[2] = z
	sample.cov_position = matrix3_eye
	sample
    end



    it 'count zero timestamp' do
        velocity_provider.apply_conf_file("uwv_kalman_filters::VelocityProvider.yml",['default'])
	Orocos.transformer.load_conf("static_transforms.rb")
	Orocos.transformer.setup(velocity_provider)

        velocity_provider.configure
        velocity_provider.start

	count_zero_timestamp = 0

        for i in 0..20
            effort = zero_command
            effort.time = effort.time + 0.01*i
            body_efforts.write effort

	    vel_data = assert_has_one_new_sample  velocity_samples, 1
	    if vel_data.time == Time.at(0)
		count_zero_timestamp = count_zero_timestamp +1
	    end

	    dvl = dvl_data(Types::Base::Vector3d.Zero)
	    dvl.time = dvl.time + 0.01*i
	    dvl_velocity_samples.write dvl

	    vel_data = assert_has_one_new_sample  velocity_samples, 1
	    if vel_data.time == Time.at(0)
		count_zero_timestamp = count_zero_timestamp +1
	    end

	    imu = imu_data(Types::Base::Vector3d.Zero)
	    imu.time = imu.time + 0.01*i
	    imu_sensor_samples.write imu

	    vel_data = assert_has_one_new_sample  velocity_samples, 1
	    if vel_data.time == Time.at(0)
		count_zero_timestamp = count_zero_timestamp +1
	    end

	    depth = depth_data(-3)
	    depth.time = depth.time + 0.01*i
	    pressure_sensor_samples.write depth

	    vel_data = assert_has_one_new_sample  velocity_samples, 1
	    if vel_data.time == Time.at(0)
		count_zero_timestamp = count_zero_timestamp +1
	    end
        end

        assert_equal 0, count_zero_timestamp

    end

    it 'count repeated timestamp' do
        velocity_provider.apply_conf_file("uwv_kalman_filters::VelocityProvider.yml",['default'])
	Orocos.transformer.load_conf("static_transforms.rb")
	Orocos.transformer.setup(velocity_provider)

        velocity_provider.configure
        velocity_provider.start

	last_timestamp = Time.at(0)
	count_repeated_timestamp = 0

        for i in 0..20
            effort = zero_command
            effort.time = effort.time + 0.01*i
            body_efforts.write effort

	    vel_data = assert_has_one_new_sample  velocity_samples, 1
	    if vel_data.time != Time.at(0) && vel_data.time == last_timestamp
		count_repeated_timestamp = count_repeated_timestamp +1
	    end
	    last_timestamp = vel_data.time

	    dvl = dvl_data(Types::Base::Vector3d.Zero)
	    dvl.time = dvl.time + 0.01*i
	    dvl_velocity_samples.write dvl

	    vel_data = assert_has_one_new_sample  velocity_samples, 1
	    if vel_data.time != Time.at(0) && vel_data.time == last_timestamp
		count_repeated_timestamp = count_repeated_timestamp +1
	    end
	    last_timestamp = vel_data.time

	    imu = imu_data(Types::Base::Vector3d.Zero)
	    imu.time = imu.time + 0.01*i
	    imu_sensor_samples.write imu

	    vel_data = assert_has_one_new_sample  velocity_samples, 1
	    if vel_data.time != Time.at(0) && vel_data.time == last_timestamp
		count_repeated_timestamp = count_repeated_timestamp +1
	    end
	    last_timestamp = vel_data.time

	    depth = depth_data(-3)
	    depth.time = depth.time + 0.01*i
	    pressure_sensor_samples.write depth

	    vel_data = assert_has_one_new_sample  velocity_samples, 1
	    if vel_data.time != Time.at(0) && vel_data.time == last_timestamp
		count_repeated_timestamp = count_repeated_timestamp +1
	    end
	    last_timestamp = vel_data.time
        end

        assert_equal 0, count_repeated_timestamp

    end

end
