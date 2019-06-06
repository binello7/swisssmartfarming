#!/bin/bash

date='190426'
file='2019-04-26-10-34-00.bag'
location='witzwil1'

cameras=(
  BFS
  Photonfocus_vis
  Ximea
)

topics=(
  /ssf/BFS_usb_0/image_raw
  /ssf/photonfocus_camera_vis_node/image_raw
  /ximea_asl/image_raw
)

path_in="/media/$USER/Samsung_2TB/Datasets/$date/$location/$file"
#path_in="$HOME/Desktop/test_bags/2019-06-05-14-37-42.bag"

for ((i=0; i<${#cameras[@]}; i++)); do
  path_out="/media/$USER/Samsung_2TB/Processed/$date/$location/launchtest/${cameras[$i]}"
  #path_out="$HOME/Desktop/test_processed/${cameras[$i]}"
  echo "Save images to folder $path_out"
  mkdir -p $path_out
  export ROS_HOME=$path_out
  roslaunch mav_startup export_jpeg.launch path:=$path_in topic:=${topics[$i]}
done
