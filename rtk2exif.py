#!/usr/bin/python2

import numpy as np
import os
import argparse

class Rtk_writer(object):
    def __init__(self):
        self.parser = argparse.ArgumentParser(description='writes rtk-data to extracted images')
        self.args = None
        self.args_parse()
        self.rtk_file = None
        self.tstamps_file = None
        self.img_folder = None

    def args_parse(self):
        self.parser.add_argument('--input_folder', required=True,
                        metavar="/my/path/to/input/images",
                        help='Path to the folder where images are stored')
        self.parser.add_argument('--rtk_file', required=True,
                        help='Path to the rtk-data file')
        self.parser.add_argument('--tstamps_file', required=True,
                        help='Path to the timestamps-data file')
        self.args = self.parser.parse_args()

    def run(self):
        self.rtk_file = self.args.rtk_file
        self.tstamps_file = self.args.tstamps_file
        self.img_folder = self.args.input_folder

        rtk_data = np.genfromtxt(self.rtk_file, delimiter=',', skip_header=1)
        tstamps_data = np.genfromtxt(self.tstamps_file, delimiter=',', skip_header=1)

        lat = np.zeros(tstamps_data.shape[0])
        lon = np.zeros(tstamps_data.shape[0])
        alt = np.zeros(tstamps_data.shape[0])

        # interpolate lat/lon/alt for every frame
        lat = np.interp(tstamps_data[:,2], rtk_data[:, 2], rtk_data[:, 6])
        lon = np.interp(tstamps_data[:,2], rtk_data[:, 2], rtk_data[:, 7])
        alt = np.interp(tstamps_data[:,2], rtk_data[:, 2], rtk_data[:, 8])

        # write exif metadata to frames
        img_names = sorted(os.listdir(self.img_folder))
        for i in range(len(img_names)):
            if img_names[i].startswith('frame_'):
                os.system("exiftool -GPSLatitudeRef=%.1f %s  -overwrite_original" %
                          (lat[i], os.path.join(self.img_folder, img_names[i])))
                os.system("exiftool -GPSLatitude=%.10f %s -overwrite_original" %
                          (lat[i], os.path.join(self.img_folder, img_names[i])))
                os.system("exiftool -GPSLongitudeRef=%.1f %s -overwrite_original" %
                          (lon[i], os.path.join(self.img_folder, img_names[i])))
                os.system("exiftool -GPSLongitude=%.10f %s -overwrite_original" %
                          (lon[i], os.path.join(self.img_folder, img_names[i])))

if __name__ == "__main__":
    rtk2exif = Rtk_writer()
    rtk2exif.run()
