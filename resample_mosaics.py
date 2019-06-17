import gdal
from PIL import Image
import numpy as np
import os
import math


def resample(img_raw, bands):
    width_px = img_raw.shape[1]
    height_px = img_raw.shape[0]
    blksize = int(math.sqrt(bands))
    offset_c = int(width_px % blksize)
    offset_r = int(height_px % blksize)
    width_px = width_px - offset_c
    height_px = height_px - offset_r
    width_px_res = int(width_px / blksize)
    height_px_res = int(height_px / blksize)

    img_gray = img_raw[:, :, 0]
    img_res = np.zeros((height_px_res, width_px_res, bands))
    band = 0
    for i in range(blksize):
        for j in range(blksize):
            img_tmp = img_gray[np.arange(i, height_px, blksize), :]
            img_res[:, :, band] = img_tmp[:, np.arange(j, width_px, blksize)]
            band += 1
    return img_res


# variables
input_folder = '/media/seba/Samsung_2TB/Processed/190426/witzwil1/Photonfocus_vis/'
output_folder = '/media/seba/Samsung_2TB/Processed/190426/witzwil1/Photonfocus_vis/Resampled/'
bands = 16

# dependent variables and operations
img_names = [f for f in sorted(os.listdir(input_folder)) if os.path.isfile(os.path.join(input_folder, f))]
if not os.path.isdir(output_folder):
    os.mkdir(output_folder)

# loop through every image in the folder
for img_name in img_names:

    # open an image
    img = np.array(Image.open((input_folder + img_name)))

    # resample the opened image
    img_res = resample(img, bands)

    # save 1 image singularly to check contrast
    contrast_folder = output_folder + 'Contrast/'
    if not os.path.isdir(contrast_folder):
        os.mkdir(contrast_folder)
    for i in range(bands):
        Image.fromarray(img_res[:, :, i]).convert("L").save((contrast_folder + 'band_' + str(i+1) + '.jpg'))

    # write GeoTiff
    dst_ds = gdal.GetDriverByName('GTiff').Create((output_folder + img_name.split('.')[0] + '.tif'),
                                                  img_res.shape[1], img_res.shape[0], bands, gdal.GDT_Byte)
    for i in range(bands):
        dst_ds.GetRasterBand(i+1).WriteArray(img_res[:, :, i])

    dst_ds.FlushCache() # write to disk
    dst_ds = None
