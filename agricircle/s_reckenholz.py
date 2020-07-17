#!/usr/bin/env python3

from datainterface import DataInterface
from rasterio.warp import reproject, Resampling
import numpy as np
import os
import rasterio as rio


root_path = '/media/seba/Samsung_2TB/matterhorn-project/qgis/reckenholz'

nir1_name = 'nir/20200608_flight20m_photonfocus_nir_georef.tif'
vis1_name = 'vis/20200608_flight20m_photonfocus_vis_georef.tif'
names = [nir1_name, vis1_name]

paths = []
for name in names:
    paths.append(os.path.join(root_path, name))

shp_path = '/media/seba/Samsung_2TB/matterhorn-project/qgis/reckenholz/shapes/field_boundary.shp'

reckenholz = DataInterface()

for path in paths:
    reckenholz.add_dataset(path)

reckenholz.add_shapefile(shp_path)
reckenholz.crop_with_shapefile()

outputs_path = '/media/seba/Samsung_2TB/matterhorn-project/qgis/reckenholz/visnir'
reckenholz.write_visnir(outputs_path)
