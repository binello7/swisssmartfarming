#!/bin/bash

date='190618'
file='2019-06-18-10-25-13.bag'
location='zollikofen'

cameras=(
  BFS
  Photonfocus_vis
  Photonfocus_nir
  #Ximea
)

topics=(
  /ssf/BFS_usb_0/image_raw
  /ssf/photonfocus_camera_vis_node/image_raw
  /ssf/photonfocus_camera_nir_node/image_raw
  #/ximea_asl/image_raw
)

path_bag="/media/$USER/Samsung_2TB/Datasets/$date/$location/$file"
path_location="/media/$USER/Samsung_2TB/Processed/$date/$location"

for ((i=0; i<${#cameras[@]}; i++)); do
  path_camera="$path_location/${cameras[$i]}"
  echo "Save images to folder $path_camera"
  mkdir -p $path_camera
  python bag2img.py --img_topic=${topics[$i]} --bag=$path_bag \
    --output_folder=$path_camera --output_format=jpg

  # Write images timestamps to file
  rostopic echo -b $path_bag -p "${topics[$i]}/header" > "$path_camera/img_tstamps.csv"
done

# Write rtk-GPS data to csv-file
rostopic echo -b $path_bag -p /ssf/dji_sdk/rtk_position > "$path_location/rtk_data.csv"
