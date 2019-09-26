#!./venv/bin/python2

import matplotlib.pyplot as plt
import numpy as np

def read_geotiff (filename):
    from osgeo import gdal_array
    img_array = gdal_array.LoadFile(filename)
    img_array = np.transpose(img_array)
    img_array = np.swapaxes(img_array, 0, 1)
    return img_array


img_file = "/home/seba/polybox/Matterhorn Project/Pix4D/niederhasli/20190719/nir/reflectance/niederhasli_20190719_nir_transparent_reflectance_group1.tif"

img_array = read_geotiff (img_file)

#plt.imshow(img_array)
