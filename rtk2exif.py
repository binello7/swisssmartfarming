#!/usr/bin/python3

import numpy as np
import os
import argparse
import timeit

class Rtk_writer(object):
    def __init__(self):
        self.parser = argparse.ArgumentParser(
            description='Writes the rtk-GPS positions recorded during the flight to the exif metadata of every image according to its timestamp'
        )
        self.args = None
        self.args_parse()
        self.rtk_file = None
        self.tstamps_file = None
        self.img_folder = None

    def args_parse(self):
        self.parser.add_argument('-i', '--input_folder', required=True,
                        help='Path to the folder where images are stored')
        self.parser.add_argument('--rtk_file', required=True,
                        help='Path to the rtk-data file')
        self.parser.add_argument('--tstamps_file', required=True,
                        help='Path to the timestamps-data file')
        self.args = self.parser.parse_args()

    def run(self):
        start = timeit.default_timer()
        self.rtk_file = self.args.rtk_file
        self.tstamps_file = self.args.tstamps_file
        self.img_folder = self.args.input_folder

        # read rtk-data and tstamps-data
        rtk_data = np.genfromtxt(self.rtk_file, delimiter=',', skip_header=1)
        tstamps_data = np.genfromtxt(self.tstamps_file, delimiter=',', skip_header=1)

        #initialize empty vectors to store lat, lon and alt infos
        lat = np.zeros(tstamps_data.shape[0])
        lon = np.zeros(tstamps_data.shape[0])
        alt = np.zeros(tstamps_data.shape[0])

        # interpolate lat/lon/alt for every frame
        lat = np.interp(tstamps_data[:,2], rtk_data[:, 2], rtk_data[:, 6])
        lon = np.interp(tstamps_data[:,2], rtk_data[:, 2], rtk_data[:, 7])
        alt = np.interp(tstamps_data[:,2], rtk_data[:, 2], rtk_data[:, 8])

        # write exif metadata to frames
        img_list = sorted(os.listdir(self.img_folder))

        ## remove files that are not images
        for img in img_list[:]: #img_list[:] makes a copy of img_list
            if not (img.startswith('frame_')):
                img_list.remove(img)

        i = 0
        n_images = len(img_list)
        for img in img_list:
            os.system("exiftool -GPSLatitudeRef=%.1f %s  -overwrite_original" %
                      (lat[i], os.path.join(self.img_folder, img)))
            os.system("exiftool -GPSLatitude=%.10f %s -overwrite_original" %
                      (lat[i], os.path.join(self.img_folder, img)))
            os.system("exiftool -GPSLongitudeRef=%.1f %s -overwrite_original" %
                      (lon[i], os.path.join(self.img_folder, img)))
            os.system("exiftool -GPSLongitude=%.10f %s -overwrite_original" %
                      (lon[i], os.path.join(self.img_folder, img)))
            os.system("exiftool -GPSAltitudeRef=%s %s -overwrite_original" %
                      ("above", os.path.join(self.img_folder, img)))
            os.system("exiftool -GPSAltitude=%.8f %s -overwrite_original" %
                      (alt[i], os.path.join(self.img_folder, img)))
            print("{}/{} files processed\n".format(i+1, n_images))
            i+=1

        end = timeit.default_timer()
        print('Processing of {} images took {:.02f}s'.format(n_images, end-start))

if __name__ == "__main__":
    rtk2exif = Rtk_writer()
    rtk2exif.run()
