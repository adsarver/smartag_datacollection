#!/bin/bash
gnome-terminal --tab -- roslaunch hardwarelaunch controller.launch
echo "Base launched"
sleep 2

gnome-terminal --tab -- roslaunch velodyne_pointcloud VLP16_points.launch
echo "Velodyne Launched"

gnome-terminal --tab -- rosrun tf static_transform_publisher 0 0 0 0 0 0 base_link lidar_link 0
echo "SLAM ready"

gnome-terminal --tab -- rviz rviz
echo "Opened Rviz"

gnome-terminal --tab -- roslaunch beemapping gmapping_demo.launch
echo "Attempted to run beemapping"