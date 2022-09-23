# UWV Kalman Filters Orogen Package

This package contains different orogen taks for different Pose Estimation components that are being used on AUVs

## Pose Estimator

Main pose estimation component. Fuses measurements from all different sensors such as IMU, DVL, Pressure Sensor, USBL and camera features

### Development Nodes:

In the Pose Estimator the PoseUKF is being used. The state of the Pose filter will be in body-aligned IMU to NWU-aligned navigation frame. That means, that measurements have to be transformed into IMU frame (translation) with body frame rotation. The navigation frame should be aligned to true-north NWU frame, so a rotation between used navigation and true north has to be provided.
