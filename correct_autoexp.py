#!./venv/bin/python2

import argparse
import numpy as np
from osgeo import gdal_array
from osgeo import gdal

# parse arguments
parser = argparse.ArgumentParser(description='Corrects image brightness ')

# read dataset exposure_times
exptimes_file = 'exptime_ximea.csv'
exp_times = np.genfromtxt(exptimes_file, delimiter=',', skip_header=1)
avg_exp_time = np.mean(exp_times)

corr_factors = exp_times/avg_exp_time

# read an images

img_folder = '/media/seba/Samsung_2TB/Processed/gorner.glacier/20190710/copy_flight2/nir'
img_file = img_folder + '/frame_002337.tif'
img_array = gdal_array.LoadFile(img_file)
img_array = np.transpose(img_array)
img_array = np.swapaxes(img_array, 0, 1)
img_array = 1.5 * img_array
print(img_array.shape)

driver = gdal.GetDriverByName('GTiff').Create((img_folder + '/frame_002337_corr.tif'), img_array.shape[1], img_array.shape[0],img_array.shape[2], gdal.GDT_Byte)
type(driver)

for i in range(img_array.shape[2]):
    driver.GetRasterBand(i+1).WriteArray(img_array[:, :, i])

driver.FlushCache() # write to disk
driver = None
