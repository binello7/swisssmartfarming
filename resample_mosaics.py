#!./venv/bin/python2

from osgeo import gdal
from PIL import Image
import numpy as np
import os
import math
import argparse
import random as rnd
import yaml
import glob
from IPython import embed
from utils.read_corr_file import read_corr_matrix


class Resampler:
    def __init__(self):
        self.parser = argparse.ArgumentParser(description='resample mosaic images to multilayer GeoTiffs')
        self.args = None
        self.args_parse()
        self.input_folder = None
        self.sep = os.path.sep
        with open('./cfg/cameras.cfg', 'r') as yml:
            try:
                self.cfg = yaml.safe_load(yml)
            except yaml.YAMLError as e:
                print(e)
        self.xml = self.cfg['photonfocus_nir']['xml_file']
        self.nb_bands = self.cfg['photonfocus_nir']['nb_bands']
        self.nb_v_bands = None

    def args_parse(self):
        self.parser.add_argument('--input_folder', required=True,
            help='Path to the folder containing the mosaic images to resample')
        self.parser.add_argument('--camera', required=True,
            help='Number of bands of the camera')
        self.parser.add_argument('--overwrite_original',
            default=False,
            help='overwrites the original non-resampled image',
            action='store_true')

        self.args = self.parser.parse_args()

    def resample(self, img_raw):
        width_px = img_raw.shape[1]
        height_px = img_raw.shape[0]
        blksize = int(math.sqrt(self.nb_bands)) #TODO: read nb_bands from config-file
        offset_c = int(width_px % blksize)
        offset_r = int(height_px % blksize)
        width_px = width_px - offset_c
        height_px = height_px - offset_r
        width_px_res = int(width_px / blksize)
        height_px_res = int(height_px / blksize)

        img_res = np.zeros((height_px_res, width_px_res, self.nb_bands))
        band = 0
        for i in range(blksize):
            for j in range(blksize):
                img_tmp = img_raw[np.arange(i, height_px, blksize), :]
                img_res[:, :, band] = img_tmp[:, np.arange(j, width_px, blksize)]
                band += 1
        return img_res
#-------------------------------------------------------------------------------

    def corr_arrange(self, img_raw):
        width_px = img_raw.shape[1]
        height_px = img_raw.shape[0]
        blksize = int(math.sqrt(self.nb_bands)) #TODO: read nb_bands from config-file
        offset_c = int(width_px % blksize)
        offset_r = int(height_px % blksize)
        width_px = width_px - offset_c
        height_px = height_px - offset_r

        # compute dimensions of reshaped image
        cols = int(width_px / blksize)
        rows = int(height_px / blksize)

        wavelengths, corr_matrix = read_corr_matrix(self.xml)
        self.nb_v_bands = wavelengths.shape[0]
        img_corr = np.zeros((rows, cols, self.nb_v_bands))
        for row in range(rows):
            start_row = row*blksize
            end_row = row*blksize+blksize
            for col in range(cols):
                start_col = col*blksize
                end_col = col*blksize+blksize
                spectrum_raw = img_raw[start_row:end_row, start_col:end_col]
                spectrum_raw = spectrum_raw.flatten()
                spectrum_corr = corr_matrix.dot(spectrum_raw)
                img_corr[row, col, :] = spectrum_corr
        img_max = np.max(img_corr)
        img_min = np.min(img_corr)
        b = 255
        a = 0
        normalize = lambda x: (b - a)* (x - img_min) / (img_max - img_min) + a
        vectorized_normalized = np.vectorize(normalize)
        img_corr = np.round(vectorized_normalized(img_corr))
        return np.array(img_corr, dtype='uint8')
#-------------------------------------------------------------------------------

    def run(self):
        self.input_folder = self.args.input_folder

        # remove os.sep if needed
        if self.input_folder.endswith(self.sep):
            self.input_folder = self.input_folder[:-1]

        # set output_folder name depending if overwrite_original is active
        if self.args.overwrite_original:
            output_folder = self.input_folder
        else:
            output_folder = self.input_folder + self.sep + 'Resampled'

        if not os.path.isdir(output_folder):
            os.mkdir(output_folder)

        img_list = glob.glob(self.input_folder + self.sep + "*.jpg")
        img_list.extend(glob.glob(self.input_folder + self.sep + "*.png"))

        # loop through every image in the folder and resample it
        for img in img_list:
            print("resampling " + img)

            if img.split('.')[-1] == 'tif':
                print('{} already resampled. Skipping frame'.format(img))
                continue

            else:
                # open the image
                img_raw = np.array(Image.open(os.path.join(self.input_folder, img)))

                # resample the opened image
                img_res = self.corr_arrange(img_raw)

                # save 1 image singularly to check contrast
                contrast_frame = img_list[int(len(img_list)/2)]

                if  img == contrast_frame:
                    contrast_folder = output_folder + '/Contrast'

                    if not os.path.isdir(contrast_folder):
                        os.mkdir(contrast_folder)

                    for i in range(self.nb_bands):
                        Image.fromarray(img_res[:, :, i]).convert("L").save((contrast_folder +
                            '/' + contrast_frame.split('.')[0] + '_band' + str(i+1) + '.jpg'))

                img_basename = img.split(self.sep)[-1].split('.')[0]
                # write GeoTiff
                dst_ds = gdal.GetDriverByName('GTiff').Create((output_folder +
                    self.sep + img_basename + '.tif'),
                    img_res.shape[1], img_res.shape[0], self.nb_v_bands, gdal.GDT_Byte)
                for i in range(self.nb_v_bands):
                    dst_ds.GetRasterBand(i+1).WriteArray(img_res[:, :, i])

                dst_ds.FlushCache() # write to disk
                dst_ds = None

                # remove the opened image if overwrite_original is on
                if self.args.overwrite_original:
                    os.remove(os.path.join(self.input_folder, img))


if __name__ == "__main__":
    resampler=Resampler()
    resampler.run()
