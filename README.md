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
  1. Navigate to the workspace src folder and run the launch_nobag.sh:
  ```
  cd ~/${YOUR_CATKIN_WS}/src
  ./launch_nobag.sh
  ```
  2. In Rviz, click *Add*, go to the *By topic* tab, add *map*, repeat for anything else you would like to visualize
  3. Start data collection:
  ```
  ./launch_bag.sh
  ```
  ***Note: If the Velodyne point cloud is flickering change the speed in the webUI to 2.4x the amount specified in the launch file
