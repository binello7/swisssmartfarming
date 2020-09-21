from data_interface import Dataset, Data_Interface
import geopandas as gpd
import matplotlib.pyplot as plt
import numpy as np
import os
import rasterio as rio
import rasterio.mask as riom
import shapely

rgb_path = ('/media/seba/Samsung_2TB/forest-project/qgis/gubler/rgb/'
            '20200626_flight2_blackfly_rgb_transparent_mosaic_group1.tif')
ms_path = ('/media/seba/Samsung_2TB/forest-project/qgis/gubler/nir/'
           '20200626_flight2_photonfocus_nir_transparent_reflectance_group1.tif')

masks_path = ('/media/seba/Samsung_2TB/forest-project/qgis/gubler/shapes/'
               'trees.shp')
boundary_path = ('/media/seba/Samsung_2TB/forest-project/qgis/gubler/shapes/'
                 'boundary.shp')

# dataset = rio.open(img_path)

# shapefile = gpd.read_file(masks_path)
# shapes = shapefile.geometry

# (img_mask, transf_mask) = riom.mask(dataset, shapes)
# img_mask = np.swapaxes(img_mask, 0, 2)
# plt.imshow(img_mask[:,:,0:3])

boundary = gpd.read_file(boundary_path)
mask = gpd.read_file(masks_path)

dataset = Dataset(
    name='gubler',
    date='20200626',
    rgb_path=rgb_path,
    ms_path=ms_path,
    mask_shapefile=mask,
    outer_shapefile=boundary,
    rgb_bands_to_read=[0, 1, 2],
    ms_bands_to_read=None,
    )
dataset = [dataset]

di_train = Data_Interface(dataset, {'tree': 1})


di_train.save('train', skip_black_greater=0.95)
