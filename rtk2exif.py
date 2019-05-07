#!/usr/bin/python2

import numpy as np
import subprocess as sp
import os

# program inputs
project_folder = "/home/seba/Desktop/20190418/"
images_folder = "xi_res/"
bag_file = "2019-04-18-14-01-03-xi.bag"
rtk_file = "140103_rtk_position.csv"
images_topic = "/ximea_asl/image_raw/header/stamp"
bands = 25

rtk_data = np.genfromtxt((project_folder + rtk_file), delimiter=',', skip_header=1)

# extract the frames' timestamps
proc = sp.Popen(["rostopic echo -b " + project_folder + bag_file + " " + images_topic],
                stdout=sp.PIPE, shell=True)

(out, err) = proc.communicate()

# store the frames' timestamps into an array
secs_nsecs = [int(s) for s in out.split() if s.isdigit()]
tstamps = np.zeros([int(len(secs_nsecs) / 2), 1])
for t in range(int(len(secs_nsecs) / 2)):
    tstamp = str(secs_nsecs[2 * t]) + '.' + str(secs_nsecs[2 * t + 1])
    tstamps[t] = float(tstamp, )

tstamps = tstamps * 1E9

# interpolate lat/lon for every frame
lat = np.interp(tstamps, rtk_data[:, 2], rtk_data[:, 6])
lon = np.interp(tstamps, rtk_data[:, 2], rtk_data[:, 7])

# write exif metadata to frames
img_files = sorted(os.listdir((project_folder + images_folder)))
for i in range(len(tstamps)):
    img_file = img_files[i*25].split('_')[0] + '*'
    os.system("exiftool -GPSLatitudeRef=%.1f %s%s%s  -overwrite_original" % (lat[i], project_folder, images_folder, img_file))
    os.system("exiftool -GPSLatitude=%.10f %s%s%s -overwrite_original" % (lat[i], project_folder, images_folder, img_file))
    os.system("exiftool -GPSLongitudeRef=%.1f %s%s%s -overwrite_original" % (lon[i], project_folder, images_folder, img_file))
    os.system("exiftool -GPSLongitude=%.10f %s%s%s -overwrite_original" % (lon[i], project_folder, images_folder, img_file))
