#!./venv/bin/python2

import numpy as np
from osgeo import gdal_array


# read dataset exposure_times
exptimes_file = 'exptime_ximea.csv'
exp_times = np.genfromtxt(exptimes_file, delimiter=',', skip_header=1)
avg_exp_time = np.mean(exp_times)

corr_factors = exp_times/avg_exp_time

# read an images
img_file = '/media/seba/Samsung_2TB/Processed/gorner.glacier/20190710/copy_flight2/nir/frame_000468.tif'
img_array = gdal_array.LoadFile(img_file)
print(img_array.shape)
