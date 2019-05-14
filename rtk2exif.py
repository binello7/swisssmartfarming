#!/usr/bin/python2

import numpy as np
import os

# program inputs
project_folder = "/home/seba/Desktop/20190418/"
images_parentfolder = "xi_res2_??/"
bag_file = "2019-04-18-14-01-03.bag"
rtk_file = "140103_rtk_position.csv"
stamps_file = "140103_stamps.txt"
images_topic = "/ximea_asl/image_raw/header/stamp"
bands = 25

rtk_data = np.genfromtxt((project_folder + rtk_file), delimiter=',', skip_header=1)

stamps_data = open((project_folder + stamps_file)).read()

# store the frames' timestamps into an array
secs_nsecs = [int(s) for s in stamps_data.split() if s.isdigit()]
tstamps = np.zeros([int(len(secs_nsecs) / 2), 1])
for t in range(int(len(secs_nsecs) / 2)):
    tstamp = str(secs_nsecs[2 * t]) + '.' + str(secs_nsecs[2 * t + 1])
    tstamps[t] = float(tstamp, )

tstamps = tstamps * 1E9

# interpolate lat/lon for every frame
lat = np.interp(tstamps, rtk_data[:, 2], rtk_data[:, 6])
lon = np.interp(tstamps, rtk_data[:, 2], rtk_data[:, 7])
alt = np.interp(tstamps, rtk_data[:, 2], rtk_data[:, 8])

# write exif metadata to frames
img_files = sorted(os.listdir((project_folder + 'xi_res2_01/')))
for i in range(len(tstamps)):
    img_file = img_files[i].split('_')[0] + '*'
    os.system("exiftool -GPSLatitudeRef=%.1f %s%s%s  -overwrite_original" %
              (lat[i], project_folder, images_parentfolder, img_file))
    os.system("exiftool -GPSLatitude=%.10f %s%s%s -overwrite_original" %
              (lat[i], project_folder, images_parentfolder, img_file))
    os.system("exiftool -GPSLongitudeRef=%.1f %s%s%s -overwrite_original" %
              (lon[i], project_folder, images_parentfolder, img_file))
    os.system("exiftool -GPSLongitude=%.10f %s%s%s -overwrite_original" %
              (lon[i], project_folder, images_parentfolder, img_file))
#    os.system('exiftool -GPSAltitudeRef="Above Sea Level" %s%s%s -overwrite_original' %
#              (project_folder, images_parentfolder, img_file))
#    os.system("exiftool -GPSAltitude=%.10f %s%s%s -overwrite_original" %
#              (alt[i], project_folder, images_parentfolder, img_file))
