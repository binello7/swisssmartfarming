import gdal
from PIL import Image
import numpy as np
import os
import math
import argparse
import random as rnd


class Resampler(object):
    def __init__(self):
        self.parser=argparse.ArgumentParser(description='resample mosaic images to multilayer GeoTiffs')
        self.args=None
        self.input_folder=None
        self.nb_bands=None
        self.args_parse()

    def args_parse(self):
        self.parser.add_argument('--input_folder', required=True,
                        metavar='/my/path/to/folder/to/resample',
                        help='Path to the folder containing the mosaic images to resample')
        self.parser.add_argument('--nb_bands', required=True,
                        help='Number of bands of the camera')
        self.args = self.parser.parse_args()

    def resample(self, img_raw):
        width_px = img_raw.shape[1]
        height_px = img_raw.shape[0]
        blksize = int(math.sqrt(self.nb_bands))
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

    def run(self):
        self.input_folder=self.args.input_folder
        output_folder=self.input_folder + '/Resampled'
        if not os.path.isdir(output_folder):
            os.mkdir(output_folder)

        self.nb_bands=int(self.args.nb_bands)

        img_names = [f for f in sorted(os.listdir(self.input_folder)) if os.path.isfile(os.path.join(self.input_folder, f))]

        # loop through every image in the folder
        for img_name in img_names:
            if img_name.startswith('frame_'):
                print("resampling image " + img_name)
                # open an image
                img = np.array(Image.open((self.input_folder + '/' + img_name)))

                # resample the opened image
                img_res = self.resample(img)

                # save 1 image singularly to check contrast
                contrast_frame = img_names[int(len(img_names)/2)]

                if img_name == contrast_frame:
                    contrast_folder = output_folder + '/Contrast'
                    if not os.path.isdir(contrast_folder):
                        os.mkdir(contrast_folder)
                    for i in range(self.nb_bands):
                        Image.fromarray(img_res[:, :, i]).convert("L").save((contrast_folder +
                                '/' + contrast_frame.split('.')[0] + '_band' + str(i+1) + '.jpg'))

                # write GeoTiff
                dst_ds = gdal.GetDriverByName('GTiff').Create((output_folder + '/' + img_name.split('.')[0] + '.tif'),
                                                          img_res.shape[1], img_res.shape[0], self.nb_bands, gdal.GDT_Byte)
                for i in range(self.nb_bands):
                    dst_ds.GetRasterBand(i+1).WriteArray(img_res[:, :, i])

                dst_ds.FlushCache() # write to disk
                dst_ds = None

if __name__ == "__main__":
    resampler=Resampler()
    resampler.run()
