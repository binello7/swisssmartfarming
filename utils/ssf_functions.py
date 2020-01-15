import numpy as np
from osgeo import gdal_array
from osgeo import gdal
import rasterio as rio
import cv2
import tkFileDialog
import Tkinter as tk
from roipoly import RoiPoly
from matplotlib import pyplot as plt
import math
import os

def read_geotiff(filepath):
    with rio.open(filepath) as src:
        img_array = src.read()

    return img_array


def write_geotiff(array, filepath):
        dst_ds = gdal.GetDriverByName('GTiff').Create(filepath,
                array.shape[1], array.shape[0], array.shape[2], gdal.GDT_Byte)

        for b in range(array.shape[2]):
            dst_ds.GetRasterBand(b+1).WriteArray(array[:, :, b])

        dst_ds.FlushCache() # write to disk
        dst_ds = None


def get_DNpanel(camera_type, initialdir="/media/seba/Samsung_2TB/Processed"):
    root = tk.Tk()
    root.withdraw()
    imgpath = tkFileDialog.askopenfilename(
        initialdir=initialdir,
        title="Choose reference panel image for {}-camera".format(camera_type))
    root.destroy()

    img = ssf.read_geotiff(imgpath)
    bands = img.shape[2]

    fig = plt.figure()
    band = 10
    plt.imshow(img[:,:,band], cmap=plt.get_cmap("Greys_r"))
    plt.colorbar()
    plt.show(block=False)

    roi = RoiPoly(color='r', fig=fig)

    mask = roi.get_mask(img[:,:,band])

    mean = np.zeros(bands)
    min = np.zeros(bands)
    max = np.zeros(bands)
    for b in range(bands):
        mean[b] = np.mean(img[:,:,b][mask])
        min[b] = np.min(img[:,:,b][mask])
        max[b] = np.max(img[:,:,b][mask])

    return mean


def argmax_2D(array_2D):
    flat_idx = np.argmax(array_2D)
    idx_1 = math.trunc(flat_idx / array_2D.shape[1])
    idx_2 = np.argmax(array_2D[idx_1,:])
    return [idx_1, idx_2]


def gdal_crop(input_file, cutline_file, Xres, Yres,
              output_file, nodata_val=0.0):
    """function wrapping the GDAL function 'gdalwarp'.

    Makes a call of the 'gdalwarp' function with the passed parameters.
    For this to be executed correctly GDAL has to be correctly installed
    on the system.

    Parameters
    ----------
    input_file : str
        path to the file to be cropped
    cutline_file : str
        path to the file to be used as cutline
    Xres : float
        desired output X-resolution
    Yres : float
        desired output Y-resolution
    output_file : str
        path and name of the output file
    nodata_val : float, optional
        pixel value to assign to regions outside the cutline. If no
        value is passed, then this is set to 0
    """

    cutline_name = cutline_file.split("/")[-1].split(".")[0]
    sys_cmd = ("gdalwarp -of GTiff -tr {} {} -tap -cutline"
               " {} -cl {} -crop_to_cutline -dstnodata {} {} {}").format(
                    Xres,
                    Yres,
                    cutline_file,
                    cutline_name,
                    nodata_val,
                    input_file,
                    output_file
                )
    print(nodata_val)
    print(type(nodata_val))
    os.system(sys_cmd)


def ndvi(nm790, nm670):
    """function returning a NDVI map.

    NDVI is calculated as:
    NDVI = (790nm - 670nm) / (790nm + 670nm)

    which corresponds to:
    NDVI = (NIR - R) / (NIR + R)

    Parameters
    ----------
    nm790 : numpy.ndarray
        2d-numpy array of the target response at 790nm (NIR)
    nm670 : numpy.ndarray
        2d-numpy array of the target response at 670nm (R)
    """

    return (nm790 - nm670) / (nm790 + nm670)


def ndre(nm790, nm720):
    """function returning a NDRE map.

    NDRE is calculated as:
    NDRE = (790nm - 720nm) / (790nm + 720nm)

    which corresponds to:
    NDRE = (NIR - RE) / (NIR + RE)

    Parameters
    ----------
    nm790 : numpy.ndarray
        2d-numpy array of the target response at 790nm (NIR)
    nm720 : numpy.ndarray
        2d-numpy array of the target response at 720nm (RE)
    """

    return (nm790 - nm720) / (nm790 + nm720)


def ccci(ndre):
    """function returning a CCCI map.

    CCCI is calculated as:
    CCCI = (NDRE - NDRE_min) / (NDRE_max - NDRE_min)

    Parameters
    ----------
    ndre : numpy.ndarray
        2d-numpy array NDRE matrix
    """
    ndre_min = np.nanmin(ndre)
    ndre_max = np.nanmax(ndre)

    return (ndre - ndre_min) / (ndre_max - ndre_min)
