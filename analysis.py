from __future__ import division
import PIL.Image as Image
import numpy as np
import matplotlib.pyplot as plt
import ssf_functions as ssf
from osgeo import gdal
#-------------------------------------------------------------------------------
from scipy import ndimage as ndi
from skimage.feature import peak_local_max

px_res = 0.00814116 # m/px

date1_path = "/media/seba/Samsung_2TB/Analysis/QGIS/niederhasli_mosaic.qgs/dsm/niederhasli_20190527_rgb_dsm_C9.tif"
date2_path = "/media/seba/Samsung_2TB/Analysis/QGIS/niederhasli_mosaic.qgs/dsm/niederhasli_20190719_rgb_dsm_C9.tif"
date3_path = "/media/seba/Samsung_2TB/Analysis/QGIS/niederhasli_mosaic.qgs/dsm/niederhasli_20191007_rgb_dsm_C9.tif"

date1_array = ssf.read_geotiff(date1_path)
date2_array = ssf.read_geotiff(date2_path)
date3_array = ssf.read_geotiff(date3_path)

# obtain field mask
field_mask = date1_array==0

# compute relative height differences from absolute altitudes
date1_array = date1_array - np.min(date1_array[date1_array!=0])
date2_array = date2_array - np.min(date2_array[date2_array!=0])
date3_array = date3_array - np.min(date3_array[date3_array!=0])

# flatten the terrain by removing the "DSM before sowing"
date2_array = date2_array - date1_array
date3_array = date3_array - date1_array

date2_orig = date2_array.copy()
date3_orig = date3_array.copy()


date3_array[date3_array<0.3] = 0

# try scipy
image_max = ndi.maximum_filter(date3_array, size=100, mode='constant')
coordinates = peak_local_max(date3_array, min_distance=100)
print(coordinates.shape)
#-------------------------------------------------------------------------------

fig, axs = plt.subplots(1, 3)
axs[0].imshow(date3_array)
axs[0].set_title('date3_array')

axs[1].imshow(image_max)
axs[1].set_title('image_max')

axs[2].imshow(date3_array)
axs[2].autoscale(False)
axs[2].plot(coordinates[:, 1], coordinates[:, 0], 'r.')
plt.show()



# plt.imshow(date3_array)
# plt.colorbar()
# plt.show()


# from scipy import ndimage as ndi
# from skimage.feature import peak_local_max
# from skimage import data, img_as_float
#
#
# # image_max is the dilation of im with a 20*20 structuring element
# # It is used within peak_local_max function
# image_max = ndi.maximum_filter(im, size=20, mode='constant')
#
# # Comparison between image_max and im to find the coordinates of local maxima
# coordinates = peak_local_max(im, min_distance=20)
#
# # display results
# fig, axes = plt.subplots(1, 3, figsize=(8, 3), sharex=True, sharey=True)
# ax = axes.ravel()
# ax[0].imshow(im, cmap=plt.cm.gray)
# ax[0].axis('off')
# ax[0].set_title('Original')
#
# ax[1].imshow(image_max, cmap=plt.cm.gray)
# ax[1].axis('off')
# ax[1].set_title('Maximum filter')
#
# ax[2].imshow(im, cmap=plt.cm.gray)
# ax[2].autoscale(False)
# ax[2].plot(coordinates[:, 1], coordinates[:, 0], 'r.')
# ax[2].axis('off')
# ax[2].set_title('Peak local max')
#
# fig.tight_layout()
#
# plt.show()




# bare_raster = gdal.Open(bare_path)
# plants_raster = gdal.Open(plants_path)
#
# bare_gt = bare_raster.GetGeoTransform()
# plants_gt = plants_raster.GetGeoTransform()
#
# print(bare_gt)
# print(plants_gt)
