# LeGO-LOAM

This repository contains code for a lightweight and ground optimized lidar odometry and mapping (LeGO-LOAM) system for ROS compatible UGVs. The system takes in point cloud  from a Velodyne VLP-16 Lidar (palced horizontal) and optional IMU data as inputs. It outputs 6D pose estimation in real-time. A demonstration of the system can be found here -> https://www.youtube.com/watch?v=O3tz_ftHV48
[![Watch the video](/LeGO-LOAM/launch/demo.gif)](https://www.youtube.com/watch?v=O3tz_ftHV48)

## Dependency

- [ROS](http://wiki.ros.org/ROS/Installation) (tested with indigo and kinetic)
- [gtsam](https://bitbucket.org/gtborg/gtsam) (Georgia Tech Smoothing and Mapping library)

## Compile

You can use the following commands to download and compile the package.

```
cd ~/catkin_ws/src
git clone https://github.com/RobustFieldAutonomyLab/LeGO-LOAM.git
cd ..
catkin_make -j1
```
When you compile the code for the first time, you need to add "-j1" behind "catkin_make" for generating some message types. "-j1" is not needed for future compiling.

## The system

LeGO-LOAM is speficifally optimized for a horizontally placed VLP-16 on a ground vehicle. It assumes there is always a ground plane in the scan. The UGV we are using is Clearpath Jackal. It has a built-in IMU. 
![Jackal](/LeGO-LOAM/launch/jackal-label.jpg)

The package performs segmentation before feature extraction.
![Segmentaion](/LeGO-LOAM/launch/seg-total.jpg)

Lidar odometry performs two-step Levenberg Marquardt optimization to get 6D transformation.
![Odometry](/LeGO-LOAM/launch/odometry.jpg)

## New sensor

The key thing to adapt the code to a new sensor is making sure the point cloud can be properly projected to an range image and ground can be correctly detected. For example, VLP-16 has a angular resolution of 0.2&deg; and 2&deg; along two directions. It has 16 beams. The angle of the bottom beam is -15&deg;. Thus, the parameters in "utility.h" are listed as below. When you implement new sensor, make sure that the ground_cloud has enough points for matching. Before you post any issues, please read this.

```
extern const int N_SCAN = 16;
extern const int Horizon_SCAN = 1800;
extern const float ang_res_x = 0.2;
extern const float ang_res_y = 2.0;
extern const float ang_bottom = 15.0;
extern const int groundScanInd = 7;
```
## Run the package

1. Run the launch file:
```
roslaunch lego_loam run.launch
```
Notes: The parameter "/use_sim_time" is set to "true" for simulation, "false" to real robot usage.

2. Play existing bag files:
```
rosbag play *.bag --clock --topic /velodyne_points /imu/data
```
Notes: Though /imu/data is optinal, it can improve estimation accuracy greatly if provided. Some sample bags can be downloaded from [here](https://github.com/RobustFieldAutonomyLab/jackal_dataset_20170608) If your IMU frame doesn't align with Velodyne frame, use of IMU data will cause significant drift.

## Cite *LeGO-LOAM*

Thank you for citing our *LeGO-LOAM* paper if you use any of this code: 
```
@inproceedings{legoloam2018,
  title={LeGO-LOAM: Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain},
  author={Tixiao Shan and Brendan Englot},
  booktitle={IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={4758-4765},
  year={2018},
  organization={IEEE}
}
```
