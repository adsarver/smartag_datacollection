# smartag_datacollection

**WIP**

# Description and goals
  Collect farm and plant data using a Clearpath Robotics Husky base from various farms in and around West Virginia.
  
# Installation
1. Start by cloning these repositories into your workspace src folder (ex. ~/catkin-ws/src):
```
git clone https://github.com/husky/husky_cartographer_navigation
```
2. Navigate back to your workspace directory and install the dependencies (ex. ~/catkin-ws):
```
cd ..
rosdep install --from-paths src --ignore-src --rosdistro=$ROS_DISTRO -y
```
3. Build the workspace:
```
catkin_make
```
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

# Exporting data into readable formats
## TODO by Mitchell
  
