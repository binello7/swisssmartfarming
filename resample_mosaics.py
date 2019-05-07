#!/usr/bin/python2

from PIL import Image
import numpy as np
import os
import matplotlib.pyplot as plt

# variables
input_folder = '/home/seba/Desktop/20190418/xi/'
output_folder = '/home/seba/Desktop/20190418/xi_res/'
sensor_type = 5  # 5 for ximea, 4 for photonfocus

# create the output folder
if not os.path.isdir(output_folder):
    os.mkdir(output_folder)

# loop through every image in the folder
img_names = sorted(os.listdir(input_folder, ))
for img_name in img_names:

    # open an image (photonfocus or ximea)
    img = np.array(Image.open((input_folder + img_name)))

    # remove pixels that are not shared by all the bands
    rm_lin = img.shape[0] % sensor_type
    rm_col = img.shape[1] % sensor_type
    height_px = img.shape[0] - rm_lin
    width_px = img.shape[1] - rm_col
    img = img[0:height_px, 0:width_px, 0]

    # resample the image
    img_resample = np.zeros((img.shape[0] / sensor_type, img.shape[1] / sensor_type, sensor_type ** 2))
    layer = 0
    for i in range(sensor_type):
        for j in range(sensor_type):
            img_temp = img[np.arange(i, height_px, sensor_type), :]
            img_resample[:, :, layer] = img_temp[:, np.arange(j, width_px, sensor_type)]

            img_jpg = Image.fromarray(img_resample[:, :, layer])
            img_jpg = img_jpg.convert("L")
            img_res_name = os.path.splitext(img_name)[0] + '_' + str(layer+1).rjust(2, '0') + '.jpg'
            print "saving image %s" % img_res_name
            img_jpg.save((output_folder + img_res_name))
            layer += 1
