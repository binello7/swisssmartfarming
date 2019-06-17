#!/bin/bash

date='190607'
file='2019-06-07-15-57-31.bag'
location='frick'

cameras=(
  BFS
  Photonfocus_vis
  Photonfocus_nir
)

topics=(
  /ssf/BFS_usb_0/image_raw
  /ssf/photonfocus_camera_vis_node/image_raw
  /ssf/photonfocus_camera_nir_node/image_raw
)

path_in="/media/$USER/Samsung_2TB/Datasets/$date/$location/$file"

for ((i=0; i<${#cameras[@]}; i++)); do
  path_out="/media/$USER/Samsung_2TB/Processed/$date/$location/${cameras[$i]}"
  echo "Save images to folder $path_out"
  mkdir -p $path_out
  export ROS_HOME=$path_out
  roslaunch mav_startup export_jpeg.launch path:=$path_in topic:=${topics[$i]}
  rm -r $path_out/log
  rm $path_out/rospack_cache_*
done
