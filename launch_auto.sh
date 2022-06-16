#!/bin/bash
gnome-terminal --tab -- roslaunch hardwarelaunch controller.launch
echo "Base launched"
sleep 2

gnome-terminal --tab -- roslaunch local_planner local_planner.launch
gnome-terminal --tab -- roslaunch terrain_analysis terrain_analysis.launch
gnome-terminal --tab -- roslaunch terrain_analysis_ext terrain_analysis_ext.launch
gnome-terminal --tab -- roslaunch sensor_scan_generation sensor_scan_generation.launch
gnome-terminal --tab -- roslaunch loam_interface loam_interface.launch
gnome-terminal --tab -- rviz rviz
echo "Opened Rviz"
sleep 2

gnome-terminal --tab -- rosrun laser_scan_matcher laser_scan_matcher_node _fixed_frame:=base_link
