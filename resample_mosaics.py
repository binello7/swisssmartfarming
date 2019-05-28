#!/usr/bin/python2

from PIL import Image
import numpy as np
import os
import georaster as gr

# variables
input_folder = '/home/seba/Desktop/20190418/xi_2/'
output_folder = '/home/seba/Desktop/20190418/'
sensor_type = 5  # 5 for ximea, 4 for photonfocus

# loop through every image in the folder
img_names = sorted(os.listdir(input_folder, ))
img_name = img_names[0]
# for img_name in img_names:

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
        band_str = str(layer+1).rjust(2, '0')
        img_res_name = os.path.splitext(img_name)[0] + '_' + band_str + '.jpg'
        # print "saving image %s" % img_res_name
        # output_subfolder = output_folder + 'xi_res2_' + band_str + '/'
        # if not os.path.isdir(output_subfolder):
        #     os.mkdir(output_subfolder)
        # img_jpg.save((output_subfolder + img_res_name))
        layer += 1


tff.imsave("testtiff.tiff", img_resample.reshape())

size=(480,640)

b1 = Image.new('RGB', size, color=10)
b2 = Image.new('RGB', size, color=20)
b3 = Image.new('RGB', size, color=30)
b4 = Image.new('RGB', size, color=40)
b5 = Image.new('RGB', size, color=50)
b6 = Image.new('RGB', size, color=60)
b7 = Image.new('RGB', size, color=70)
b8 = Image.new('RGB', size, color=80)

# Save all 8 to single TIFF file
b1.save('multi.tif', save_all=True, append_images=[b2,b3,b4,b5,b6,b7,b8])

raster = input_folder + 'frame0248.jpg'
data = gr