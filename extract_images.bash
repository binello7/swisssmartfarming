#!/bin/bash

date='190426'
file='2019-04-26-10-34-00.bag'
location='witzwil1'

cameras=(
  BFS
  Photonfocus_vis
  #Photonfocus_nir
  Ximea
)

topics=(
  /ssf/BFS_usb_0/image_raw
  /ssf/photonfocus_camera_vis_node/image_raw
  #/ssf/photonfocus_camera_nir_node/image_raw
  /ximea_asl/image_raw
)

path_in="/media/$USER/Samsung_2TB/Datasets/$date/$location/$file"

for ((i=0; i<${#cameras[@]}; i++)); do
  path_out="/media/$USER/Samsung_2TB/Processed/$date/$location/${cameras[$i]}"
  echo "Save images to folder $path_out"
  mkdir -p $path_out
  python bag2img.py --img_topic=${topics[$i]} --bag=$path_in \
    --output_folder=$path_out --output_format=jpg
done
