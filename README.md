# Error State EKF for GNSS - IMU fusion

![Notes.](/images/notes.png)

In a standard EKF, if an estimate is far from truth, linearization errors will be large. In an ESEKF approach, linearization is about the error state which is small and therefore its Jacobian captures the linearized dynamics well. Advantageously, attitude in the error state can be represented using 3 parameter state instead of a quaternion. Error state covariance remains well-conditioned as it brings all the state errors to similar numerical scales. 

But this also comes with additional logic to inject the error state into a nominal state (standard EKF - we only work with the nominal state). The state and covariance also needs to be reset after update to constrain the error growth (respects linearity assumption). 

**Note**: Reset the state and covariance - we have a new reference point to capture the error state uncertainty from. This reset transformation needs to be applied for the covariance. This says that the new attitude error uncertainty is expressed locally with respect to the attitude of the current nominal state.


This logic is tested on sports_field drone dataset: [GVINS](https://github.com/HKUST-Aerial-Robotics/GVINS-Dataset/tree/main)

Workspace: ROS2 Foxy + Ubuntu 20.04

Result:Odometry from GPS (red) vs filter(green)

![Odometry from GPS (red) vs filter(green).](/images/result0.png)

## Build Instructions

1. Download dataset and convert rosbags
``bash
rosbags-convert sports_field.bag
```
2. Clone this repo
```bash
cd /ros2_ws/src
git clone https://github.com/ram-bhaskara/GNSS-IMU-ErrorStateEKF.git
```
3. Colcon Build
```
cd /ros2_ws
colcon build --packages-select gnss_imu_esekf --cmake-clean-cache
source ../ros2_ws/install/setup.bash
```
4. Run node or the launch file
```
ros2 run  gnss_imu_esekf esekf_node
```

References:
1. Quaternion kinematics and ESKF: https://arxiv.org/pdf/1711.02508
2. ESKF and other flavors of KF: https://arxiv.org/pdf/2406.06427
