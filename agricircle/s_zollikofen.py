#!/usr/bin/env python3

from datainterface import DataInterface
import os
import rasterio as rio

from IPython import embed


nir_path = ('/media/seba/WD_500GB/zollikofen/20190705/photonfocus_nir/'
    'zollikofen_20190705_photonfocus_nir_transparent_reflectance_group1.tif')
vis_path = ('/media/seba/WD_500GB/zollikofen/20190705/photonfocus_vis/'
    'zollikofen_20190705_photonfocus_vis_transparent_reflectance_group1.tif')

wrong_path = ('/media/seba/WD_500GB/zollikofen/20190705/photonfocus_vis/'
    'zollikofen_20190605_photonfocus_vis_transparent_reflectance_group1.tif')

shp_path = '/media/seba/WD_500GB/zollikofen/qgis/shapes/field.shp'


zollikofen = DataInterface()
zollikofen.add_dataset(nir_path)
zollikofen.add_dataset(vis_path)

zollikofen.add_shapefile(shp_path)
zollikofen.clip_with_shapefile()

data, profile = zollikofen.merge_vis_nir('20190705-photonfocus_vis', '20190705-photonfocus_nir')

filepath = '/media/seba/WD_500GB/zollikofen/20190705/new.tif'
with rio.open(filepath, 'w', **profile) as dst:
    dst.write(data)



# embed()
