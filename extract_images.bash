!#/bin/bash

date='190426'
file='2019-04-26-10-34-00.bag'
location='witzwil1'


path=/media/$USER/'Samsung 2TB'/Datasets/$date/$location/$file






roscore
rosrun image_view extract_images _sec_per_frame:=0.01 image:=/ssf/BFS_usb_0/image_raw
rosbag play 2019-04-18-14-01-03.bag
