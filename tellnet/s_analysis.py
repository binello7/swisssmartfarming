#!../venv/bin/python2

import utils.ssf_functions as ssf
import os.path
import matplotlib.pyplot as plt
import numpy as np
import rasterio as rio
from rasterio import mask as msk
import fiona as fio
from IPython import embed

base_path = ("/media/seba/Samsung_2TB/TELLnet/meier-burkard/20190507/"
    "meier-burkard_20190507/4_index/reflectance/")

ndvi_path = ("/media/seba/Samsung_2TB/TELLnet/meier-burkard/20190507/"
    "meier-burkard_20190507/4_index/indices/ndvi/"
    "meier-burkard_20190507_index_ndvi.tif")

shp_path = ("/media/seba/Samsung_2TB/Analysis/QGIS/TELLnet/meier-burkard/"
    "Shapes/02_Ruetmatt.shp")

r_name = "meier-burkard_20190507_transparent_reflectance_red.tif"
g_name = "meier-burkard_20190507_transparent_reflectance_green.tif"
b_name = "meier-burkard_20190507_transparent_reflectance_blue.tif"
nir_name = "meier-burkard_20190507_transparent_reflectance_nir.tif"
re_name = "meier-burkard_20190507_transparent_reflectance_rededge.tif"

file_names = [r_name, g_name, b_name, nir_name, re_name]
paths = []

for file_name in file_names:
    paths.append(os.path.join(base_path, file_name))

# open Ruetmatt shapefile
with fio.open(shp_path, "r") as shapefile:
    ruetmatt = [feature["geometry"] for feature in shapefile]

# open r-channel and crop it with the shapefile
with rio.open(paths[0]) as src:
    r_image, r_transform = msk.mask(src, ruetmatt, crop=True)
    r_meta = src.meta

# open nir-channel and crop it with the shapefile
with rio.open(paths[3]) as src:
    nir_image, nir_transform = msk.mask(src, ruetmatt, crop=True)
    nir_meta = src.meta

# open re-channel and crop it with the shapefile
with rio.open(paths[4]) as src:
    re_image, re_transform = msk.mask(src, ruetmatt, crop=True)
    re_meta = src.meta


# "squeeze" the images
nir_image = np.squeeze(nir_image)
re_image = np.squeeze(re_image)
r_image = np.squeeze(r_image)

# set no-data values to np.nan
nir_image[nir_image==-10000] = np.nan
re_image[re_image==-10000] = np.nan
r_image[r_image==-10000] = np.nan

# compute ndvi
ndvi = ssf.ndvi(nir_image, r_image)
ndre = ssf.ndre(nir_image, nm720=re_image)
ccci = ssf.ccci(ndre)

ccci_ndvi = ccci * ndvi

indexes = [ndvi, ccci, ccci_ndvi]


# r = ssf.read_geotiff(paths[0])
# nir = ssf.read_geotiff(paths[3])
# r[r==-10000] = np.nan
# nir[nir==-10000] = np.nan
#
# ndvi = ssf.ndvi(nir, r)
#
# print(np.nanmax(ndvi))
# print(np.nanmin(ndvi))
#
# embed()


fig, axs = plt.subplots(1, len(indexes))
for n, index in enumerate(indexes):
    g = axs[n].imshow(index)
    cbar = fig.colorbar(g, ax=axs[n], shrink=0.5)
# cbar.ax.tick_params(labelsize=7)
# plt.title("NDVI Meier-Burkard 2019/05/07", fontsize=12)
# plt.xticks([])
# plt.yticks([])
# plt.box(False)
# plt.savefig("ndvi_meier-burkard_20190507.png", dpi=300)
plt.show()
