#!../venv/bin/python2

from IPython import embed
import cv2
import numpy as np
import rasterio as rio
import rasterio.plot as riop
import rasterio.transform as riot
import fiona as fio
import swisssmartfarming.utils.ssf_functions as ssf
import matplotlib.pyplot as plt
import georaster as gr
import geopandas as gpd
from swisssmartfarming.datasets.datasetmanipulator import DatasetManipulator

tifpath = "/media/seba/Samsung_2TB/Analysis/QGIS/Niederhasli/tmp/test_rgb.tif"
tifpath2 = "/media/seba/Samsung_2TB/Analysis/QGIS/Niederhasli/mosaics/niederhasli_20190719_rgb_transparent_mosaic_group1_reprojected.tif"
shppath = ("/media/seba/Samsung_2TB/Analysis/QGIS/Niederhasli"
    "/shapes/mask_plants.shp")

contour = ("/media/seba/Samsung_2TB/Analysis/QGIS/Niederhasli/shapes/cut_test.shp")
contour = gpd.read_file(contour)

imgpath = "/home/seba/Pictures/Screenshot from 2019-12-10 10-34-55.png"
img = cv2.imread(imgpath)


ds_manipulator = DatasetManipulator(tifpath)

ds_manipulator.create_grid(contour)
ds_manipulator.pad_geotiff_from_grid()
ds_manipulator.create_mask_from_shapes(shppath)

poly = ds_manipulator.grid["geometry"]

fig, axs = plt.subplots()
riop.show(ds_manipulator.dataset, ax=axs)
riop.show(ds_manipulator.mask, ax=axs, alpha=0.5)
for i, p in enumerate(poly):
    x, y = p.exterior.xy
    plt.plot(x, y)
    xm = (x[1] - x[0]) / 2
    ym = (y[2] - y[1]) / 2
    text = str(ds_manipulator.grid['grid_idx'][i])
    plt.text(x[0] + xm, y[0] + ym, text, color='r')

img, mask = ds_manipulator.get_pair_from_idx(34)
plt.show()




plt.figure()
plt.imshow(img)
plt.imshow(mask, alpha=0.5)
plt.show()



#
# mask = gpd.read_file(shppath)
# print(mask.head())
#
# # open dataset and shapefile
#
# shapes = fio.open(shppath)
#
#
# for shape in shapes:
#     print(len(shape['geometry']['coordinates'][0]))
#     x = []
#     y = []
#     for i in range(len(shape['geometry']['coordinates'][0])):
#         x.append(shape['geometry']['coordinates'][0][i][0])
#         y.append(shape['geometry']['coordinates'][0][i][1])
#
#
# rows, cols = riot.rowcol(dataset.transform, x, y, precision=None)
#
# rows = [int(r) for r in rows]
# cols = [int(c) for c in cols]
