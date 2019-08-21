#!/bin/bash

bag='2019-06-07-11-35-26.bag'
field='witzwil2'
date='190607'

cameras=(
  rgb
  vis
  #nir
  nir
  #thermal
)

bands=(
  3
  16
  #25
  25
  #1
)

topics=(
  /ssf/BFS_usb_0/image_raw
  /ssf/photonfocus_camera_vis_node/image_raw
  #/ssf/photonfocus_camera_nir_node/image_raw
  /ximea_asl/image_raw
  #/ssf/thermalgrabber_ros/image_deg_celsius
)
