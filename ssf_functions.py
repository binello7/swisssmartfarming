import numpy as np
from osgeo import gdal_array
from osgeo import gdal
import cv2
import tkFileDialog
import Tkinter as tk
from roipoly import RoiPoly
from matplotlib import pyplot as plt
import ssf_functions as ssf

def read_geotiff(filepath):
    img_array = gdal_array.LoadFile(filepath)
    img_array = np.transpose(img_array)
    img_array = np.swapaxes(img_array, 0, 1)
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
