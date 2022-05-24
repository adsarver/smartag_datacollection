# smartag_datacollection

**WIP**

# Description and goals
  Collect farm and plant data using a Clearpath Robotics Husky base from various farms in and around West Virginia.
  
# Installation
1. Start by cloning these repositories into your workspace src folder (ex. ~/catkin-ws/src):
```
git clone https://github.com/husky/husky
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

  
