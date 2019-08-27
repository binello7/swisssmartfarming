#!/bin/bash

bag='2019-07-09-21-19-08.bag'
date='190709'
flight=''

cameras=(
  rgb
  vis
  nir
  #nir
  thermal
)

bands=(
  3
  16
  25
  #25
  1
)

topics=(
  /ssf/BFS_usb_0/image_raw
  /ssf/photonfocus_camera_vis_node/image_raw
  /ssf/photonfocus_camera_nir_node/image_raw
  #/ximea_asl/image_raw
  /ssf/thermalgrabber_ros/image_deg_celsius
)
