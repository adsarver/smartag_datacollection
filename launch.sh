#!/bin/bash
gnome-terminal --tab -- roslaunch hardwarelaunch controller.launch
echo "Base launched"
sleep 3

gnome-terminal --tab -- roslaunch hardwarelaunch gps.launch
echo "GPS Launched"

gnome-terminal --tab -- roslaunch realsense2_camera rs_camera.launch tf_prefix:=realsense
echo "Realsense Launched"
sleep 2

rosbag record -o /media/bramblebee/smartag/2022-Data/ -a