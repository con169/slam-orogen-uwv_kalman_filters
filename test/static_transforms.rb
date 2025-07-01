### Sensors

## imu pose in body frame
static_transform Eigen::Quaternion.Identity,
    Eigen::Vector3.new( 0.29424, 0.0, 0.02863 ),
    "imu" => "body"

## pressure sensor in body frame
static_transform Eigen::Quaternion.Identity,
    Eigen::Vector3.new( 0.0, 0.0, -0.3 ),
    "pressure_sensor" => "body"

## dvl in body frame
static_transform Eigen::Quaternion.from_euler(Eigen::Vector3.new(22.5 / 180.0 * Math::PI, 0, 0), 2, 1, 0),
    Eigen::Vector3.new( 0.659, 0.141, -0.206 ),
    "dvl" => "body"
