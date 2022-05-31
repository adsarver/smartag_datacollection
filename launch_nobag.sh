#!/bin/bash
gnome-terminal --tab -e "roslaunch hardwarelaunch controller.launch" 
sleep 2
gnome-terminal --tab -e "roslaunch velodyne_pointcloud VLP16_points.launch"
gnome-terminal --tab -e "rosrun tf static_transform_publisher 0 0 0 0 0 0 base_link velodyne 0"
gnome-terminal --tab -e "rosrun gmapping slam_gmapping"
gnome-terminal --tab -e "rosrun laser_scan_matcher laser_scan_matcher_node _fixed_frame:=odom"
gnome-terminal --tab -e "rviz rviz"