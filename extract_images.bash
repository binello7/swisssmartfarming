#!/bin/bash

date='190426'
file='2019-04-26-10-34-00.bag'
location='witzwil1'

camera='Photonfocus_vis'
cameras=(
  BFS
  Photonfocus_vis
  Ximea
)
topic='/ssf/photonfocus_camera_vis_node/image_raw'
topics=(
  /ssf/BFS_usb_0/image_raw
  /ssf/photonfocus_camera_vis_node/image_raw
  /ximea_asl/image_raw
)

path_in="/media/$USER/Samsung_2TB/Datasets/$date/$location/$file"

for ((i=0; i<${#cameras[@]}; i++)); do
  path_out="/media/$USER/Samsung_2TB/Processed/$date/$location/${cameras[$i]}"
  echo "Extract images to folder $path_out"
  mkdir -p $path_out
  export ROS_HOME=$path_out
  roslaunch mav_startup export_jpeg.launch path:=$path_in topic:=${topics[$i]}
done
