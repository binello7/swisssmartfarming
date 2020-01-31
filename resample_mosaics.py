#!./venv/bin/python2

from osgeo import gdal
from PIL import Image
import numpy as np
import os
import math
import argparse
import random as rnd

def resample(mosaic, nb_bands):
    width_px = mosaic.shape[1]
    height_px = mosaic.shape[0]
    blksize = int(math.sqrt(nb_bands))
    offset_c = int(width_px % blksize)
    offset_r = int(height_px % blksize)
    width_px = width_px - offset_c
    height_px = height_px - offset_r
    width_px_res = int(width_px / blksize)
    height_px_res = int(height_px / blksize)

    img_res = np.zeros((height_px_res, width_px_res, nb_bands))
    band = 0
    for i in range(blksize):
        for j in range(blksize):
            img_tmp = mosaic[np.arange(i, height_px, blksize), :]
            img_res[:, :, band] = img_tmp[:, np.arange(j, width_px, blksize)]
            band += 1
    return img_res

class Resampler:
    def __init__(self, args):
        self.args=args
        self.nb_bands=None

    def run(self):
        self.input_folder = self.args.input_folder

        # set output_folder name depending if overwrite_original is active
        if self.args.overwrite_original:
            output_folder = self.input_folder
        else:
            output_folder = self.input_folder + '/Resampled'

        if not os.path.isdir(output_folder):
            os.mkdir(output_folder)

        self.nb_bands = int(self.args.nb_bands)

        img_list = sorted(os.listdir(self.input_folder))

        # remove files that are not images
        for img in img_list[:]: #img_list[:] makes a copy of img_list
            if not (img.startswith('frame_')):
                img_list.remove(img)

        # loop through every image in the folder and resample it
        for img in img_list:
            print("resampling " + img)

            if img.split('.')[-1] == 'tif':
                print('{} already resampled. Skipping frame'.format(img))
                continue

            else:
                # open an image
                img_raw = np.array(Image.open(os.path.join(self.input_folder, img)))

                # resample the opened image
                img_res = self.resample(img_raw)

                # save 1 image singularly to check contrast
                contrast_frame = img_list[int(len(img_list)/2)]

                if  img == contrast_frame:
                    contrast_folder = output_folder + '/Contrast'

                    if not os.path.isdir(contrast_folder):
                        os.mkdir(contrast_folder)

                    for i in range(self.nb_bands):
                        Image.fromarray(img_res[:, :, i]).convert("L").save((contrast_folder +
                                    '/' + contrast_frame.split('.')[0] + '_band' + str(i+1) + '.jpg'))

                # write GeoTiff
                dst_ds = gdal.GetDriverByName('GTiff').Create((output_folder + '/' + img.split('.')[0] + '.tif'),
                                                              img_res.shape[1], img_res.shape[0], self.nb_bands, gdal.GDT_Byte)
                for i in range(self.nb_bands):
                    dst_ds.GetRasterBand(i+1).WriteArray(img_res[:, :, i])

                dst_ds.FlushCache() # write to disk
                dst_ds = None

                # remove the opened image if overwrite_original is on
                if self.args.overwrite_original:
                    os.remove(os.path.join(self.input_folder, img))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='resample mosaic images to multilayer GeoTiffs')
    parser.add_argument('--input_folder', required=True,
        help='Path to the folder containing the mosaic images to resample')
    parser.add_argument('--nb_bands', required=True,
        help='Number of bands of the camera')
    parser.add_argument('--overwrite_original',
        default=False,
        help='overwrites the original non-resampled image',
        action='store_true')

    args = parser.parse_args()
    resampler=Resampler(args)
    resampler.run()
