#!/usr/bin/env python3

from datainterface import DataInterface
from rasterio.warp import reproject, Resampling
import numpy as np
import os
import rasterio as rio


root_path = '/media/seba/Samsung_2TB/matterhorn-project/qgis/reckenholz/'

files = [
    'nir/20200608_flight20m_photonfocus_nir_georef.tif',
    'nir/20200623_flight20m_photonfocus_nir_georef.tif',
    'nir/20200630_flight20m_photonfocus_nir_georef.tif',
    'vis/20200608_flight20m_photonfocus_vis_georef.tif',
    'vis/20200623_flight20m_photonfocus_vis_georef.tif',
    'vis/20200630_flight20m_photonfocus_vis_georef.tif'
]
filepaths = []
for file in files:
    filepaths.append(os.path.join(root_path, file))

shp_path = ('/media/seba/Samsung_2TB/matterhorn-project/qgis/reckenholz/shapes/'
    'field_boundary.shp')

reckenholz = DataInterface()

for filepath in filepaths:
    reckenholz.add_dataset(filepath)

reckenholz.add_shapefile(shp_path)

outputs_path = ('/media/seba/Samsung_2TB/matterhorn-project/qgis/reckenholz/'
    'visnir')

reckenholz.crop_merge_write_visnir(outputs_path)
