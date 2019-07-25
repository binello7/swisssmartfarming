#!/usr/bin/python3

import numpy as np
import os
import argparse

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
        self.rtk_file = self.args.rtk_file
        self.tstamps_file = self.args.tstamps_file
        self.img_folder = self.args.input_folder
        focal_length = 12.5
        fstop = "f/2.8"
        maker = "FLIR"
        model = "Blackfly-S"
        metering_mode = "Average"
        exp_time = 0.002
        aperture_value = "2.8"
        fnumber = "2.8"
        equivalent35 = "51.8"

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
        n_files = 0
        for img_name in img_names:
            if img_name.startswith('frame_'):
                n_files+=1

        for i in range(len(img_names)):
            if img_names[i].startswith('frame_'):
                # os.system("exiftool -GPSLatitudeRef=%.1f %s  -overwrite_original" %
                #           (lat[i], os.path.join(self.img_folder, img_names[i])))
                # os.system("exiftool -GPSLatitude=%.10f %s -overwrite_original" %
                #           (lat[i], os.path.join(self.img_folder, img_names[i])))
                # os.system("exiftool -GPSLongitudeRef=%.1f %s -overwrite_original" %
                #           (lon[i], os.path.join(self.img_folder, img_names[i])))
                # os.system("exiftool -GPSLongitude=%.10f %s -overwrite_original" %
                #           (lon[i], os.path.join(self.img_folder, img_names[i])))
                # os.system("exiftool -GPSAltitudeRef=%s %s -overwrite_original" %
                #           ("above", os.path.join(self.img_folder, img_names[i])))
                # os.system("exiftool -GPSAltitude=%.8f %s -overwrite_original" %
                #           (alt[i], os.path.join(self.img_folder, img_names[i])))
                # os.system("exiftool -GPSAltitude=%.8f %s -overwrite_original" %
                #           (alt[i], os.path.join(self.img_folder, img_names[i])))
                os.system("exiftool -FocalLength=%s %s -overwrite_original" %
                          (focal_length, os.path.join(self.img_folder, img_names[i])))
                os.system("exiftool -ApertureValue=%s %s -overwrite_original" %
                          (aperture_value, os.path.join(self.img_folder, img_names[i])))
                os.system("exiftool -FNumber=%s %s -overwrite_original" %
                          (fnumber, os.path.join(self.img_folder, img_names[i])))
                os.system("exiftool -make=%s %s -overwrite_original" %
                          (maker, os.path.join(self.img_folder, img_names[i])))
                os.system("exiftool -model=%s %s -overwrite_original" %
                          (model, os.path.join(self.img_folder, img_names[i])))
                os.system("exiftool -FocalLengthIn35mmFormat=%s %s -overwrite_original" %
                          (equivalent35, os.path.join(self.img_folder, img_names[i])))
                print("%d/%d files processed" % (i+1, n_files))

if __name__ == "__main__":
    rtk2exif = Rtk_writer()
    rtk2exif.run()
