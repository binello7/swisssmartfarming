#!/usr/bin/env python3

from datainterface import DataInterface
from glob import glob
import os

from IPython import embed


# Input folder
# mosaics_folder = ("/media/seba/WD_500GB/tellnet/datasets/mueller/anglikon/"
#     "20200425/mosaics")
mosaics_folder = ("/media/seba/WD_500GB/tellnet/datasets/mueller/anglikon/"
    "20200507/mosaics")

# Shapefile
shp_path = ('/media/seba/WD_500GB/tellnet/datasets/mueller/anglikon/shapes/'
    'anglikon.shp')

# Read files in mosaics_folder
datasets_paths = glob(os.path.join(mosaics_folder, '*.tif'))

anglikon = DataInterface()

# Add all dataset mosaics
for dataset in datasets_paths:
    anglikon.add_dataset(dataset)

# Add shapefile
anglikon.add_shapefile(shp_path)

# Crop datasets with shapefile
anglikon.crop_with_shapefile()

date = '20200507'
anglikon.set_ndvi(date)
anglikon.write_ndvi(mosaics_folder, date)
