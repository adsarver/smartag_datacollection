#!/bin/bash
gnome-terminal --tab -- roslaunch hardwarelaunch controller.launch
echo "Base launched"
sleep 2

gnome-terminal --tab -- rviz rviz
echo "Opened Rviz"
sleep 2

gnome-terminal --tab -- roslaunch hardwarelaunch VelodyneEdit.launch
gnome-terminal --tab -- rosrun laser_scan_matcher laser_scan_matcher_node _fixed_frame:=base_link
echo "Velodyne Launched"

gnome-terminal --tab -- roslaunch hardwarelaunch zed2i.launch
echo "Zed 2i Launched"

gnome-terminal --tab -- roslaunch realsense2_camera rs_camera.launch tf_prefix:=realsense
echo "Realsense Launched"

gnome-terminal --tab -- roslaunch beemapping gmapping_demo.launch
echo "Attempted to run beemapping"

gnome-terminal --tab -- roslaunch hardwarelaunch gps.launch
echo "Launched GPS publisher"
