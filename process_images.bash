#!/bin/bash

source dataset_infos.bash

if [[ $flight == ""  ]]
then
  path_bag="/media/$USER/Samsung_2TB/Datasets/gorner.glacier/$date/$bag"
  path_out="/media/$USER/Samsung_2TB/Processed/gorner.glacier/20$date"
else
  path_bag="/media/$USER/Samsung_2TB/Datasets/gorner.glacier/$date/$bag"
  path_out="/media/$USER/Samsung_2TB/Processed/gorner.glacier/20$date/$flight"
fi

mkdir -p $path_out

# Write rtk-GPS data to csv-file
./rtk2csv.py --bag_file $path_bag --output_folder $path_out

for ((i=0; i<${#cameras[@]}; i++))
do
  path_camera="$path_out/${cameras[$i]}"
  echo "Save images to folder $path_camera"
  mkdir -p $path_camera

  if [[ ${cameras[$i]} == "thermal" ]]
  then
    ./thermal2tiff.py --bag_file $path_bag --output_folder $path_camera
  else
    python bag2img.py --img_topic=${topics[$i]} --bag=$path_bag \
      --output_folder=$path_camera --output_format=jpg
  fi

  if [[ ${cameras[$i]} == "nir" ]] || [[ ${cameras[$i]} == "vis" ]]
  then
    ./resample_mosaics.py --input_folder=$path_camera --nb_bands=${bands[$i]} --overwrite_original
  fi

  # Write images timestamps to csv-file
  ./tstamps2csv.py --topic ${topics[$i]} --bag_file $path_bag --output_folder $path_camera

  # Write rtk-GPS data to exif metadata
  ./rtk2exif.py -i $path_camera --rtk_file "$path_out/rtk_data.csv" \
    --tstamps_file "$path_camera/img_tstamps.csv"
done

echo -e '\nProcessing of dataset completed successfully'
