#!/bin/bash

project='gorner_glacier'
bag='2019-06-19-12-38-48.bag'
date='rheinau'

cameras=(
  BFS
  Photonfocus_vis
  Photonfocus_nir
  #Ximea
)

bands=(
  3
  16
  25
  #25
)

topics=(
  /ssf/BFS_usb_0/image_raw
  /ssf/photonfocus_camera_vis_node/image_raw
  /ssf/photonfocus_camera_nir_node/image_raw
  #/ximea_asl/image_raw
)

path_bag="/media/hdd/$USER/bags/$project/$date/$bag"
path_imgs="/media/hdd/$USER/img_temp/$project/$date"

# Write rtk-GPS data to csv-file
echo "Saving rtk-GPS data..."
rostopic echo -b $path_bag -p /ssf/dji_sdk/rtk_position > "$path_imgs/rtk_data.csv"

for ((i=0; i<${#cameras[@]}; i++)); do
  path_camera="$path_imgs/${cameras[$i]}"
  echo "Save images to folder $path_camera"
  mkdir -p $path_camera
  python bag2img.py --img_topic=${topics[$i]} --bag=$path_bag \
    --output_folder=$path_camera --output_format=jpg
  if [[ ${cameras[$i]} == "Ximea" ]] || [[ ${cameras[$i]} == "Photonfocus_vis" ]] || [[ ${cameras[$i]} == "Photonfocus_nir" ]]
  then
    python resample_mosaics.py --input_folder=$path_camera --nb_bands=${bands[$i]}
  fi

  # Write images timestamps to file
  echo "Saving images timestamps..."
  rostopic echo -b $path_bag -p "${topics[$i]}/header" > "$path_camera/img_tstamps.csv"

  # Write rtk-GPS data to exif metadata
  if [[ ${cameras[$i]} == "BFS" ]]
  then
    ./rtk2exif.py -i $path_camera --rtk_file $path_imgs/rtk_data.csv \
      --tstamps_file $path_camera/img_tstamps.csv
  fi
done
