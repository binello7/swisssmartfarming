from __future__ import division
import PIL.Image as Image
import numpy as np
import matplotlib.pyplot as plt
import ssf_functions as ssf
from osgeo import gdal

bare_path = "/media/seba/Samsung_2TB/Analysis/niederhasli_bare_field_rgb_dsm_crop.tif"
plants_path = "/media/seba/Samsung_2TB/Analysis/niederhasli_20191007_rgb_dsm_crop.tif"

bare_array = ssf.read_geotiff(bare_path)
plants_array = ssf.read_geotiff(plants_path)

min_diff = np.min(bare_array[bare_array!=0]) - np.min(plants_array[plants_array!=0])

bare_array = bare_array - min_diff
plants_array[1176:1257,4592:4663] = bare_array[1176:1257,4592:4663]

diff_array = plants_array - bare_array



# bare_array = bare_array - min_diff
#
# max_diff = np.max(plants_array) - np.max(bare_array)



# bare_raster = gdal.Open(bare_path)
# plants_raster = gdal.Open(plants_path)
#
# bare_gt = bare_raster.GetGeoTransform()
# plants_gt = plants_raster.GetGeoTransform()
#
# print(bare_gt)
# print(plants_gt)
