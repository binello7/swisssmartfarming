#!/bin/bash

# ./script --bag_path=/path/to/bag --thermal_cam=yes|no
# --nir_cam=photonfocus|ximea

export PYTHONPATH=/usr/lib/python2.7/dist-packages:${PYTHONPATH}

# cameras parameters
cameras=(
  rgb
  vis
  nir
)

bands=(
  3
  16
  25
)

topics=(
  /ssf/BFS_usb_0/image_raw
  /ssf/photonfocus_camera_vis_node/image_raw
  /ssf/photonfocus_camera_nir_node/image_raw
)

# input arguments
filepath=$1
readarray -d '/' -t filepath_array <<< "$filepath"

date=${filepath_array[5]}
field=${filepath_array[6]}

idx_last=${#filepath_array[@]}
idx_last=$((idx_last - 1))
bag=${filepath_array[$idx_last]}

# param2:if thermal camera was used: yes|no. Default yes
# param3: which nir camera: ximea|photonfocus. Default photonfocus

# set thermal_cam parameters (param2)
if [[ $2 == "" ]] || [[ $2 == "yes" ]]
then
  cameras[3]="thermal"
  bands[3]=1
  topics[3]="/ssf/thermalgrabber_ros/image_deg_celsius"
elif [[ $2 == "no" ]]
then
  true
else
  echo "Wrong value for <thermal_cam> parameter. Value can be either 'yes' or 'no'"
  exit 1
fi

# set nir_cam parameters (param3)
cameras[2]="nir"
bands[2]=25
if [[ $3 == "" ]] || [[ $3 == "photonfocus" ]]
then
  topics[2]="/ssf/photonfocus_camera_nir_node/image_raw"
elif [[ $3 == "ximea" ]]
then
  topics[2]="/ximea_asl/image_raw"
else
  echo "Wrong value for <nir_cam> parameter. Value can be either 'photonfocus' or 'ximea'"
  exit 1
fi

path_bag="/media/$USER/Samsung_2TB/Datasets/$date/$field/$bag"
path_date="/media/$USER/Samsung_2TB/Processed/$field/20$date"

mkdir -p $path_date

# Write rtk-GPS data to csv-file
./rtk2csv.py --bag_file $path_bag --output_folder $path_date

for ((i=0; i<${#cameras[@]}; i++))
do
  path_camera="$path_date/${cameras[$i]}"
  echo "Save images to folder $path_camera"
  mkdir -p $path_camera

  if [[ ${cameras[$i]} == "thermal" ]]
  then
    ./thermal2tiff.py --bag_file $path_bag --output_folder $path_camera
  else
    ./bag2img.py --topic=${topics[$i]} --bag_file=$path_bag \
      --output_folder=$path_camera --output_format=png
  fi

  if [[ ${cameras[$i]} == "nir" ]] || [[ ${cameras[$i]} == "vis" ]]
  then
    ./resample_mosaics.py --input_folder=$path_camera --nb_bands=${bands[$i]} --overwrite_original
  fi

  # Write images timestamps to csv-file
  ./tstamps2csv.py --topic ${topics[$i]} --bag_file $path_bag --output_folder $path_camera

  # Write rtk-GPS data to exif metadata
  ./write_exif.py -i $path_camera --rtk_file "$path_date/rtk_data.csv" \
    --tstamps_file "$path_camera/img_tstamps.csv"
done

echo -e '\nProcessing of dataset completed'
