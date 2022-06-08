#!/bin/bash
gnome-terminal --tab -- roslaunch hardwarelaunch controller.launch
echo "Base launched"
sleep 2
gnome-terminal --tab -- roslaunch velodyne_pointcloud VLP16_points.launch
echo "Velodyne Launched"
gnome-terminal --tab -- rosrun tf static_transform_publisher 0 0 0 0 0 0 base_link velodyne 0
gnome-terminal --tab -- rosrun gmapping slam_gmapping
gnome-terminal --tab -- rosrun laser_scan_matcher laser_scan_matcher_node _fixed_frame:=odom
gnome-terminal --tab -- docker pull stereolabs/zed:[tag]
gnome-terminal --tab -- docker run --gpus all -it --privileged stereolabs/zed:[tag]
echo "ZED SDK Container"
gnome-terminal --tab -- roslaunch zed_wrapper zed2i.launch
echo "Zed 2i Launched"
echo "SLAM ready"
gnome-terminal --tab -- rviz rviz
echo "Opened Rviz"
