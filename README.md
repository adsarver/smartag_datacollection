# smartag_datacollection

**WIP**

# Description and goals
  Collect farm and plant data using a Clearpath Robotics Husky base from various farms in and around West Virginia.
  
# Installation
1. Start by cloning this into your workspace src folder (ex. ~/catkin-ws/src):
```
git clone https://github.com/nickcharron/waypoint_nav
```
2. Install GeographicLib:
```
sudo apt update
sudo apt install geographiclib-tools
```
3. Navigate back to your workspace directory (ex. ~/catkin-ws) and install robot localization then install all of the dependencies :
```
cd ..
rosdep install robot_localization
rosdep install --from-paths src --ignore-src -r -y
```
4. Build the workspace:
```
catkin_make
```
If it fails because of *GeographicLib/MGRS.hpp: no such file or directory* then try this:

# How to run
1. Open 3 terminal windows in your catkin workspace:
```
cd ~/*INSERT_WORKSPACE_NAME*
```
2. Source the workspace in each window:
```
source devel/setup.bash
```
3. Launch the simulation, SLAM, Catographer node, and Rviz:
```
roslaunch husky_gazebo husky_playpen.launch

roslaunch husky_cartographer_navigation cartographer_demo.launch

roslaunch husky_viz view_robot.launch
```

# Setting up data collection
## TODO by Andy
1. rosbag record -a
# Exporting data into readable formats
## TODO by Mitchell
  
