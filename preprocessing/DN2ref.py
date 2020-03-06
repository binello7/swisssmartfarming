#!../venv3/bin/python3

import numpy as np
import os
import argparse
import utils.functions as ufunc
import glob
import warnings
from osgeo import gdal


parser = argparse.ArgumentParser()
parser.add_argument('--camera_path', required=True,
    help=('Path to the camera folder (nir or vis cameras) containing the '
        'frames to convert to reflectance.'))
args = parser.parse_args()


refl_path = os.path.join(args.camera_path, 'refl')
if not os.path.isdir(refl_path):
    os.makedirs(refl_path)

pathname = os.path.join(args.camera_path, '*.tif')

imgs_list = glob.glob(pathname)

mean_white, white_img = ufunc.get_avg_val_roi(args.camera_path)
exp_t_white = ufunc.get_exp_t_ms(white_img)
refl_white = 0.18

for img in imgs_list:
    exp_t_img = ufunc.get_exp_t_ms(img)
    img_array = ufunc.read_geotiff(img)
    img_refl = ufunc.rad_to_refl(img_array, exp_t_img, exp_t_white, mean_white,
        refl_white)
    min_refl = np.min(img_refl)
    max_refl = np.max(img_refl)
    if max_refl > 1:
        warnings.warn('Attention: max reflectance > 1.0: max_refl={}'.format(
            max_refl))

    img_basename = ufunc.get_file_basename(img)
    img_basename = '.'.join(img_basename)
    img_fullname = os.path.join(refl_path, img_basename)
    print("Writing '{}'".format(img_fullname))
    ufunc.write_geotiff(img_refl, img_fullname, dtype=gdal.GDT_Float32)
