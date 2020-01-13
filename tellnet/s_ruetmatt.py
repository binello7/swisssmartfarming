#!../venv/bin/python2

import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from datainterface import DataInterface
from IPython import embed
import rasterio.mask as riom
import georaster as gr

# Inputs
shapefile = "/media/seba/Samsung_2TB/TELLnet/Shapes/ruetmatt/ruetmatt.shp"
base_folder = ("/media/seba/Samsung_2TB/TELLnet/Fields/meier-burkard/"
    "buenzmatt-ruetmatt")
datasets = [
    "20190418/buenzmatt-ruetmatt_20190418_transparent_reflectance_red_georef.tif",
    "20190418/buenzmatt-ruetmatt_20190418_transparent_reflectance_green_georef.tif",
    "20190418/buenzmatt-ruetmatt_20190418_transparent_reflectance_blue_georef.tif",
    "20190418/buenzmatt-ruetmatt_20190418_transparent_reflectance_nir_georef.tif",
    "20190418/buenzmatt-ruetmatt_20190418_transparent_reflectance_rededge_georef.tif",
    "20190507/buenzmatt-ruetmatt_20190507_transparent_reflectance_red_georef.tif",
    "20190507/buenzmatt-ruetmatt_20190507_transparent_reflectance_green_georef.tif",
    "20190507/buenzmatt-ruetmatt_20190507_transparent_reflectance_blue_georef.tif",
    "20190507/buenzmatt-ruetmatt_20190507_transparent_reflectance_nir_georef.tif",
    "20190507/buenzmatt-ruetmatt_20190507_transparent_reflectance_rededge_georef.tif",
]

ruetmatt = DataInterface()
for dataset in datasets:
    ruetmatt.add_dataset(os.path.join(base_folder, dataset))

ruetmatt.add_shapefile(shapefile)
datasets_names = ruetmatt.datasets_names

for dataset_name in datasets_names:
    ruetmatt.crop_dataset(dataset_name)

ruetmatt.align_datasets(datasets_names[0])

ruetmatt.load_dataset_to_memory(datasets_names[0])

ndvi_20190418 = ruetmatt.ndvi("20190418")
ndvi_20190507 = ruetmatt.ndvi("20190507")

ndvi_20190418[~ruetmatt.data_mask] = np.nan
ndvi_20190507[~ruetmatt.data_mask] = np.nan

# generate plot
fig, axs = plt.subplots(nrows=1, ncols=2)
axs[0].imshow(ndvi_20190418)
axs[1].imshow(ndvi_20190507)
plt.show()

diff = ndvi_20190507 - ndvi_20190418

plt.imshow(diff, cmap="RdYlGn")
plt.show()
