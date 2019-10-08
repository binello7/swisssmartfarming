import numpy as np
from osgeo import gdal_array

def read_geotiff(filepath):
    img_array = gdal_array.LoadFile(filepath)
    img_array = np.transpose(img_array)
    img_array = np.swapaxes(img_array, 0, 1)
    return img_array
