# smartag_datacollection
  **WIP**
  
  Collect farm and plant data using a Clearpath Robotics Husky base from various farms in and around West Virginia.
  
## Required Hardware

 ZED 2i
 
 Velodyne VLP-16
 
 Intel Realsense D435
 
 Clearpath Robotics Husky

## Installation
  1. Make sure that the required hardware is setup properly
  2. Clone the following workspaces into your src folder and run catkin_make:
  ```
  cd ~/${YOUR_CATKIN_WS}/src
  git clone https://github.com/adsarver/smartag_datacollection
  git clone https://github.com/swri-robotics/novatel_gps_driver
  git clone https://github.com/IntelRealSense/realsense-ros
  git clone https://github.com/ros-drivers/velodyne
  git clone https://github.com/stereolabs/zed-ros-wrapper
  catkin_make
  ```
## How to run
  1. Navigate to the workspace src folder and run the launch.sh:
  ```
  cd ~/${YOUR_CATKIN_WS}/src
  sh launch.sh
  ```
  ***Note: If the Velodyne point cloud is flickering change the speed in the webUI to 2.4x the amount specified in the launch file
  
## Optional
   1. CMU Exploration: Efficient autonomous navigation and path planning
   ```
   https://www.cmu-exploration.com/development-environment
   ```
   2. FAST_LIO: Fast mapping using lidar and imu data (post and live processing)
   ```
   https://github.com/hku-mars/FAST_LIO
   ```
  
