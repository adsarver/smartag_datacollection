#!/bin/bash
gnome-terminal --tab -- roslaunch hardwarelaunch controller.launch
echo "Base launched"
sleep 2

gnome-terminal --tab -- rviz rviz
echo "Opened Rviz"
sleep 2

gnome-terminal --tab -- roslaunch realsense2_camera rs_camera.launch tf_prefix:=realsense
echo "Realsense Launched"

gnome-terminal --tab -- roslaunch hardwarelaunch gps.launch
echo "Launched GPS publisher"
