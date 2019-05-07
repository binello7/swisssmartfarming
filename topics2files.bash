#!/bin/bash

directory="/home/seba/Desktop/20190418/"
file="2019-04-18-14-01-03.bag"

full_file="$directory$file"


rostopic echo -b $full_file -p /ssf/dji_sdk/rtk_position > ${directory}140103_rtk_position.csv
