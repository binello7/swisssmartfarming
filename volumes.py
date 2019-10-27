from __future__ import division
import PIL.Image as Image
import numpy as np
import matplotlib.pyplot as plt
import ssf_functions as ssf
from osgeo import gdal
from scipy import ndimage as ndi
from skimage.feature import peak_local_max
import os



dsms_folder = "/media/seba/Samsung_2TB/Analysis/QGIS/Niederhasli/dsm/dsms_C"
dsms_files = os.listdir(dsms_folder)
dsms_files = sorted(dsms_files)

# declare variables for final results
volumes = np.zeros((len(plots), len(dates)-1))
h_max = np.zeros(volumes.shape)

dates = []
plots = []
for dsm in dsms_files:
    date = dsm.split("_")[1]
    plot = dsm.split(".")[0].split("_")[-2:]
    plot = "_".join(plot)
    plots.append(plot)
    dates.append(date)

dates = sorted(list(set(dates)))
plots = sorted(list(set(plots)))

for n_plot, plot in enumerate(plots):
    for j, date in enumerate(dates):
        dsms_file = "niederhasli_{}_rgb_dsm_{}.tif".format(date, plot)
        filepath = os.path.join(dsms_folder, dsms_file)
        dsm = ssf.read_geotiff(filepath)
        raster = gdal.Open(filepath)
        X_res = raster.GetGeoTransform()[1]
        Y_res = -raster.GetGeoTransform()[5]
        n_dates = len(dates)
        if ((j+n_dates%n_dates==0)):
            mask_plot = dsm!=0
            dsms = np.zeros(dsm.shape + (n_dates,))

        dsms[:,:,j] = dsm

        # verify if last available date for given plot
        if ((j+1)%n_dates==0):
            for i in range(n_dates):
                min_dsm = np.min(dsms[:,:,i][mask_plot])
                dsms[:,:,i][mask_plot] = dsms[:,:,i][mask_plot]-min_dsm

            dsm_ref = dsms[:,:,0].copy()

            for i in range(n_dates-1):
                dsms[:,:,i+1] = dsms[:,:,i+1] - dsm_ref
                # by subtracting dsm_ref some values get < 0. Set them to 0
                dsms[:,:,i+1][dsms[:,:,i+1]<0] = 0
                max_dsm = np.max(dsms[:,:,i+1][mask_plot])
                h_max[n_plot, i] = max_dsm
                volumes[n_plot, i] = np.sum(dsms[:,:,i+1]) * X_res * Y_res


# for i in range(n_plots):


# date1_path = "/media/seba/Samsung_2TB/Analysis/QGIS/Niederhasli/dsm/niederhasli_20190527_rgb_dsm_C9.tif"
# date2_path = "/media/seba/Samsung_2TB/Analysis/QGIS/Niederhasli/dsm/niederhasli_20190719_rgb_dsm_C9.tif"
# date3_path = "/media/seba/Samsung_2TB/Analysis/QGIS/Niederhasli/dsm/niederhasli_20191007_rgb_dsm_C9.tif"
#
# date1_array = ssf.read_geotiff(date1_path)
# date2_array = ssf.read_geotiff(date2_path)
# date3_array = ssf.read_geotiff(date3_path)
#
# # obtain field mask
# field_mask = date1_array!=0
#
# # compute relative height differences from absolute altitudes
# date1_array = date1_array - np.min(date1_array[field_mask])
# date2_array = date2_array - np.min(date2_array[field_mask])
# date3_array = date3_array - np.min(date3_array[field_mask])
#
# # set all cells outside field perimeter to 0
# date1_array[~field_mask] = 0
# date2_array[~field_mask] = 0
# date3_array[~field_mask] = 0
#
# # flatten the terrain by removing the "DSM before sowing"
# date2_flat = date2_array - date1_array
# date3_flat = date3_array - date1_array
# print(np.max(date2_flat), np.max(date3_flat))
#
# # exclude undersown crops
# date2_flat[date2_flat<0.04] = 0
# date3_flat[date3_flat<0.2] = 0
#
# image_max = ndi.maximum_filter(date2_flat, size=30, mode='constant')
# c = peak_local_max(date2_flat, min_distance=74)
#
# # remove coordinates with same value
# count = 1
# coordinates = np.ones(c.shape) * -1
# coordinates[0] = c[0]
# for i in range(c.shape[0]-1):
#     if date2_flat[c[i+1,0],c[i+1,1]] != date2_flat[c[i,0],c[i,1]]:
#         coordinates[count] = c[i+1]
#         count+=1
# coordinates = coordinates[coordinates!=-1]
# coordinates = np.reshape(coordinates, (int(coordinates.size/2), 2))
# print(coordinates.shape)
#
# # compute volume
# tot_vol = np.sum(date3_flat) * px_res**2
# print(tot_vol)
#
# plant_vol = tot_vol / nb_plants
# print(plant_vol)
#
# fig, axs = plt.subplots(1, 2)
#
# axs[1].imshow(image_max)
# axs[1].plot(coordinates[:, 1], coordinates[:, 0], 'r.')
# axs[1].set_title('image_max')
#
# axs[0].imshow(date2_flat)
# axs[0].autoscale(False)
# axs[0].plot(coordinates[:, 1], coordinates[:, 0], 'r.')
#
# fig, axs = plt.subplots(1,2)
# axs[0].imshow(date2_flat)
# axs[1].imshow(date3_flat)
# plt.show()



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
