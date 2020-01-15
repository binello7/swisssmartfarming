#!../venv/bin/python2

from config import Config
from swisssmartfarming.datasets.datasetmanipulator import DatasetManipulator
import geopandas as gpd
import numpy as np
import matplotlib.pyplot as plt
import cv2
import os

from IPython import embed

geotiff_rgb = ("/media/seba/Samsung_2TB/Analysis/QGIS/Niederhasli/tmp/"
    "field1_part_rgb.tif")
field_boundary = ("/media/seba/Samsung_2TB/Analysis/QGIS/Niederhasli/"
    "shapes/cut_test.shp")
mask_plants = ("/media/seba/Samsung_2TB/Analysis/QGIS/Niederhasli/"
    "shapes/mask_plants.shp")

config = Config()

# generate the datasetmanipulator object
ds_manip = DatasetManipulator(geotiff_rgb)

# generate the slicing grid based on the field_boundary
field_boundary = gpd.read_file(field_boundary)
ds_manip.create_grid(field_boundary)

# pad the dataset
ds_manip.pad_geotiff_from_grid()

ds_manip.create_mask_from_shapes(mask_plants)

ds_manip.visualize_dataset(with_grid=True)

if not os.path.isdir(config.train_images_path):
    os.mkdir(config.train_images_path)

if not os.path.isdir(config.train_annotations_path):
    os.mkdir(config.train_annotations_path)

for idx in ds_manip.grid.grid_idx:
    img, mask = ds_manip.get_pair_from_idx(idx)
    if np.count_nonzero(img[:,:,0]) is not 0:
        img_name = "img_" + str(idx) + ".png"
        mask_name = img_name
        img_name = os.path.join(config.train_images_path, img_name)
        mask_name = os.path.join(config.train_annotations_path, mask_name)
        cv2.imwrite(img_name, img)
        cv2.imwrite(mask_name, mask)
