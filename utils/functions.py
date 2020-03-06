import numpy as np
from osgeo import gdal
from tkinter import filedialog
import tkinter as tk
from roipoly import RoiPoly
from matplotlib import pyplot as plt
import math
import os
import pyexiv2 as px2
from IPython import embed

sep = os.path.sep

def add_sep(path):
    if not path.endswith(sep):
        path = path + sep
    return path
#-------------------------------------------------------------------------------

def rm_sep(path):
    if path.endswith(sep):
        path = path[:-1]
    return path
#-------------------------------------------------------------------------------

def get_file_basename(filepath):
    basename = filepath.split(sep)[-1]
    extension = basename.split('.')[-1]
    basename = '.'.join(basename.split('.')[:-1])
    return (basename, extension)
#-------------------------------------------------------------------------------

def read_geotiff(img_path):
    raster = gdal.Open(img_path)
    img_array = raster.ReadAsArray()
    img_array = np.moveaxis(img_array, 0, 2)
    return img_array
#-------------------------------------------------------------------------------

def write_geotiff(img_array, img_path, dtype=gdal.GDT_Byte):
    driver = gdal.GetDriverByName('GTiff')
    if len(img_array.shape) == 3:
        dataset = driver.Create(
            img_path,
            img_array.shape[1],
            img_array.shape[0],
            img_array.shape[2],
            dtype
        )
        for b in range(img_array.shape[2]):
            dataset.GetRasterBand(b+1).WriteArray(img_array[:, :, b])

    elif len(img_array.shape) == 2:
        dataset = driver.Create(
            img_path,
            img_array.shape[1],
            img_array.shape[0],
            dtype
        )
        dataset.GetRasterBand(1).WriteArray(img_array)

    else:
        raise ValueError(("'img_array' can have either shape=(h, w) or shape="
            "(h, w, b). Shape={} is not admitted.").format(img_array.shape))

    dataset.FlushCache() # write to disk
#-------------------------------------------------------------------------------

def get_avg_val_roi(initialdir):
    root = tk.Tk()
    root.withdraw()
    img_path =  filedialog.askopenfilename(initialdir=initialdir,
        title="Select reference panel image")
    root.destroy()

    img = read_geotiff(img_path)
    bands = img.shape[2]
    band = int(bands / 2)

    fig = plt.figure()
    plt.imshow(img[:,:,band], cmap=plt.get_cmap("Greys_r"))
    plt.show(block=False)

    roi = RoiPoly(color='r', fig=fig)

    mask = roi.get_mask(img[:,:,band])
    mean = np.mean(img[mask], axis=0)
    return mean, img_path
#-------------------------------------------------------------------------------

def get_exp_t_ms(img_path):
    metadata = px2.ImageMetadata(img_path)
    metadata.read()
    exp_t = metadata.get_exposure_data()['speed']
    exp_t = float(exp_t) * 1e3
    return exp_t
#-------------------------------------------------------------------------------

def rad_to_refl(img, exp_t_img, exp_t_white, mean_white, refl_white):
    img_refl = (img / mean_white) * (exp_t_white / exp_t_img) * refl_white
    return img_refl

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
