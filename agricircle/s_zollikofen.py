#!/usr/bin/env python3

from datainterface import DataInterface
import os
import rasterio as rio


root_path = '/media/seba/WD_500GB/zollikofen/qgis/rasters'

nir1_name = 'zollikofen_20190626_photonfocus_nir_transparent_reflectance_group1.tif'
vis1_name = 'zollikofen_20190626_photonfocus_vis_transparent_reflectance_group1.tif'
nir2_name = 'zollikofen_20190705_photonfocus_nir_transparent_reflectance_group1.tif'
vis2_name = 'zollikofen_20190705_photonfocus_vis_transparent_reflectance_group1.tif'
names = [nir1_name, vis1_name, nir2_name, vis2_name]

paths = []
for name in names:
    paths.append(os.path.join(root_path, name))

shp_path = '/media/seba/WD_500GB/zollikofen/qgis/shapes/field.shp'


zollikofen = DataInterface()

for path in paths:
    zollikofen.add_dataset(path)

zollikofen.add_shapefile(shp_path)
zollikofen.clip_with_shapefile()

merged_datasets, wavelengths = zollikofen.merge_vis_nir()

for (name, dataset), wls in zip((merged_datasets.items()), wavelengths):
    DataInterface.write_dataset(root_path, name, dataset, wavelengths=wls)
