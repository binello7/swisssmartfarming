#!/bin/bash

date='190426'
bag='2019-04-26-11-13-32.bag'
location='witzwil2'

cameras=(
  BFS
  Photonfocus_vis
  #Photonfocus_nir
  Ximea
  #Tau2
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
