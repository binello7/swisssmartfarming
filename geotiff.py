#!/usr/bin/env python
from osgeo import gdal
from osgeo import osr
import numpy as np

#  Initialize the Image Size
image_size = (400,400)

#  Choose some Geographic Transform (Around Lake Tahoe)
lat = [39,38.5]
lon = [-120,-119.5]

#  Create Each Channel
r_pixels = np.zeros((image_size), dtype=np.uint8)
g_pixels = np.zeros((image_size), dtype=np.uint8)
b_pixels = np.zeros((image_size), dtype=np.uint8)

#  Set the Pixel Data (Create some boxes)
for x in range(0,image_size[0]):
    for y in range(0,image_size[1]):
        if x < image_size[0]/2 and y < image_size[1]/2:
            r_pixels[y,x] = 255
        elif x >= image_size[0]/2 and y < image_size[1]/2:
            g_pixels[y,x] = 255
        elif x < image_size[0]/2 and y >= image_size[1]/2:
            b_pixels[y,x] = 255
        else:
            r_pixels[y,x] = 255
            g_pixels[y,x] = 255
            b_pixels[y,x] = 255

# set geotransform
nx = image_size[0]
ny = image_size[1]
xmin, ymin, xmax, ymax = [min(lon), min(lat), max(lon), max(lat)]
xres = (xmax - xmin) / float(nx)
yres = (ymax - ymin) / float(ny)
geotransform = (xmin, xres, 0, ymax, 0, -yres)

# create the 3-band raster file
dst_ds = gdal.GetDriverByName('GTiff').Create('myGeoTIFF.tif', ny, nx, 3, gdal.GDT_Byte)

dst_ds.SetGeoTransform(geotransform)    # specify coords
srs = osr.SpatialReference()            # establish encoding
srs.ImportFromEPSG(3857)                # WGS84 lat/long
dst_ds.SetProjection(srs.ExportToWkt()) # export coords to file
dst_ds.GetRasterBand(1).WriteArray(r_pixels)   # write r-band to the raster
dst_ds.GetRasterBand(2).WriteArray(g_pixels)   # write g-band to the raster
dst_ds.GetRasterBand(3).WriteArray(b_pixels)   # write b-band to the raster
dst_ds.FlushCache()                     # write to disk
dst_ds = None
