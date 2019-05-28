#!/bin/bash

date='190426'
file='2019-04-26-10-34-00.bag'
location='witzwil1'
camera='Photonfocus_vis'

path_in="/media/$USER/Samsung_2TB/Datasets/$date/$location/$file"
path_out="/media/$USER/Samsung_2TB/Processed/$date/$location/$camera"

echo "Creating directory $path_out"
mkdir -p $path_out

export ROS_HOME=$path_out

roslaunch photonfocus_camera export_jpeg.launch


# rosrun image_view extract_images _sec_per_frame:=0.01 image:=/ssf/BFS_usb_0/image_raw
# rosbag play 2019-04-18-14-01-03.bag
