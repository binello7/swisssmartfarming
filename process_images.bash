#!/bin/bash

source dataset_infos.bash

path_bag="/media/$USER/Samsung_2TB/Datasets/$date/$location/$bag"
path_location="/media/$USER/Samsung_2TB/Processed/$date/$location"

mkdir -p $path_location

# Write rtk-GPS data to csv-file
./rtk2csv.py --bag_file $path_bag --output_folder $path_location

for ((i=0; i<${#cameras[@]}; i++))
do
  path_camera="$path_location/${cameras[$i]}"
  echo "Save images to folder $path_camera"
  mkdir -p $path_camera

  if [[ ${cameras[$i]} == "Tau2" ]]
  then
    ./thermal2tiff.py --bag_file $path_bag --output_folder $path_camera
  else
    python bag2img.py --img_topic=${topics[$i]} --bag=$path_bag \
      --output_folder=$path_camera --output_format=jpg
  fi

  if [[ ${cameras[$i]} == "Ximea" ]] || [[ ${cameras[$i]} == "Photonfocus_vis" ]] || [[ ${cameras[$i]} == "Photonfocus_nir" ]]
  then
    ./resample_mosaics.py --input_folder=$path_camera --nb_bands=${bands[$i]} --overwrite_original
  fi

  # Write images timestamps to csv-file
  ./tstamps2csv.py --topic ${topics[$i]} --bag_file $path_bag --output_folder $path_camera

  # Write rtk-GPS data to exif metadata
  ./rtk2exif.py -i $path_camera --rtk_file "$path_location/rtk_data.csv" \
    --tstamps_file "$path_camera/img_tstamps.csv"
done

echo -e '\nProcessing of dataset completed successfully'
