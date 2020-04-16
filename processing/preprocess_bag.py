#!/usr/bin/env python2


import os
import glob
from preprocessing import BasePreprocessor, SpectralProcessor
from PIL import Image
import utils.functions as ufunc
import argparse
import textwrap
from osgeo import gdal
from IPython import embed


class MultilineFormatter(argparse.HelpFormatter):
    def _fill_text(self, text, width, indent):
        text = self._whitespace_matcher.sub(' ', text).strip()
        paragraphs = text.split('|n ')
        multiline_text = ''
        for paragraph in paragraphs:
            formatted_paragraph = textwrap.fill(paragraph, width,
                initial_indent=indent, subsequent_indent=indent) + '\n\n'
            multiline_text = multiline_text + formatted_paragraph
        return multiline_text
#===============================================================================

def compose_filepath(folder_path, prefix, file_nb, extension):
    file_name = prefix + '_{:05d}'.format(file_nb) + extension
    file_path = os.path.join(folder_path, file_name)
    return file_path
#-------------------------------------------------------------------------------

sep = os.path.sep

parser = argparse.ArgumentParser(
    description=
        """Preprocesses an SSF-rosbag dataset.
        |n
        Given a rosbag file contianing a dataset for the Swiss Smart Farming
        Project performs the preprocesses steps in order to produce
        georeferenced corrected images that can be fed to the Pix4D
        software.""",
    formatter_class=MultilineFormatter
)
parser.add_argument('--bagfile', '-b',
    required=True,
    help='Path to the bag file and name, e.g. ./dataset/bagfile.bag')
parser.add_argument('--ref_panel', '-r',
    choices=['1', '2'],
    required=True,
    help="Reference panel used to calibrate the hyperspectral images. "
        "Use '1' for 'colorChecker' and '2' for 'SphereOptics'.")

args = parser.parse_args()

# set the value of the ref panel
if args.ref_panel == '1':
    reflectance = 0.18
elif args.ref_panel == '2':
    reflectance = 0.9645

# create the BasePreprocessor object
basepreprocessor = BasePreprocessor(args.bagfile)

# set image properties
prefix = 'frame'

# set the root folder where the images will be stored
date_folder = sep.join(args.bagfile.split(sep)[:-2])
frames_folder = os.path.join(date_folder, 'frames')
if not os.path.isdir(frames_folder):
    os.makedirs(frames_folder)

# loop over cameras and process them
for cam in basepreprocessor.cams:
    # Create one folder per camera
    print("Processing camera '{}'".format(cam))
    camera_folder = os.path.join(frames_folder, cam)
    if not os.path.isdir(camera_folder):
        os.makedirs(camera_folder)

    basepreprocessor.set_cam_info(cam)

    msgs = basepreprocessor.read_img_msgs(basepreprocessor.imgs_topics[cam])
    for i, msg in enumerate(msgs):
        # get one image after another with its timestamp and process it
        img_tstamp = msg.timestamp.to_nsec()
        img_array = basepreprocessor.imgmsg_to_cv2(msg)

        # gps-rtk data are read automatically. Set img_info attribute
        basepreprocessor.set_img_info(img_tstamp)

        # Do different processing steps depending on the camera type
        if basepreprocessor.cam_info['type'] == 'RGB':
            # set filepath
            extension = '.jpg'
            filepath = compose_filepath(camera_folder, prefix, i, extension)
            # save image and write exif metadata
            im = Image.fromarray(img_array)
            im.save(filepath, quality=100)
            basepreprocessor.write_exif(filepath)

        elif basepreprocessor.cam_info['type'] == 'hyperspectral':
            # set filepath
            extension = '.tif'
            filepath = compose_filepath(camera_folder, prefix, i, extension)
            # reshape the raw sensor data
            img_array = basepreprocessor.reshape_hs(img_array)
            # apply median filtering
            img_array = basepreprocessor.median_filter_3x3(img_array)
            # save image and write exif metadata
            ufunc.write_geotiff(img_array, filepath)
            basepreprocessor.write_exif(filepath)

        elif basepreprocessor.cam_info['type'] == 'thermal':
            # set filepath
            extension = '.tif'
            filepath = compose_filepath(camera_folder, prefix, i, extension)
            # save image and write exif metadata
            im = Image.fromarray(img_array)
            im.save(filepath)
            basepreprocessor.write_exif(filepath)

# create the SpectralProcessor object
spectralprocessor = SpectralProcessor(frames_folder)

for cam in spectralprocessor.cams:
    spectralprocessor.set_cam_info(cam)
    if  spectralprocessor.is_hyperspectral:
        imgs_list = glob.glob(os.path.join(spectralprocessor.cam_folder, '*'))
        spectralprocessor.set_white_info(white_reflectance=reflectance)
        for img_path in imgs_list:
            exif = spectralprocessor.read_exif(img_path)
            img_exp_t = spectralprocessor.read_exp_t_ms(img_path)
            img_array = ufunc.read_img2array(img_path)
            # compute reflectance image
            img_refl = spectralprocessor.rad_to_refl(img_array, img_exp_t)
            # apply spectral correction
            img_corr = spectralprocessor.corr_spectra(img_refl)
            print("Writing file {}.".format(img_path))
            ufunc.write_geotiff(img_corr, img_path, dtype=gdal.GDT_Float32)
            spectralprocessor.write_exif(img_path, exif)
