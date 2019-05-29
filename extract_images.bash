#!/bin/bash

date='190426'
file='2019-04-26-10-34-00.bag'
location='witzwil1'
camera='Photonfocus_vis'
topic='/ssf/photonfocus_camera_vis_node/image_raw'

path_in="/media/$USER/Samsung_2TB/Datasets/$date/$location/$file"
path_out="/media/$USER/Samsung_2TB/Processed/$date/$location/$camera"



echo "Creating directory $path_out"
mkdir -p $path_out

export ROS_HOME=$path_out

echo $path_in
echo $topic
roslaunch mav_startup export_jpeg.launch path:=$path_in topic:=$topic
