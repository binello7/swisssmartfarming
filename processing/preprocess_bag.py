#!/usr/bin/env python3

from osgeo import gdal
from PIL import Image
from preprocessing import BasePreprocessor, SpectralProcessor
import argparse
import glob
import os
import textwrap
import utils.functions as ufunc


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
        Given a rosbag file of a dataset for the Swiss Smart Farming
        Project performs the preprocessing steps on the raw images.
        The preprocessed images will be stored under the folder 'frames',
        located one level higher than the bagfile.
        Preprocessing steps include:
        |n
        1. save the RGB images in 'jpg' format and embed the image
        metadata (camera name, focal length, GPS location, ...)
        |n
        2. Blablabla
        software.""",
    formatter_class=MultilineFormatter
)
parser.add_argument('--bagfile', '-b',
    required=True,
    help='Path to the bag file and name, e.g. ./dataset/bagfile.bag')
parser.add_argument('--ref_panel', '-r',
    choices=['1', '2', '3'],
    required=True,
    help="Reference panel used to calibrate the hyperspectral images. "
        "Use '1' for 'colorChecker' (r=0.18), '2' for 'SphereOptics' "
        "(r=0.9645), '3' for a custom panel (reflectance value asked during "
        "execution)")

args = parser.parse_args()

# set the value of the ref panel
if args.ref_panel == '1':
    reflectance = 0.18
elif args.ref_panel == '2':
    reflectance = 0.9645
elif args.ref_panel == '3':
    reflectance = float(input("Enter the average reflectance value of the "
            "used calibration panel (must be > 0, <= 1): "))
    while reflectance <=0 or reflectance > 1:
        reflectance = float(input("Wrong value. Reflectance value must be > 0, "
            "<= 1. Please enter it again: "))

# create the BasePreprocessor object
basepreprocessor = BasePreprocessor(args.bagfile)

# set image properties
prefix = 'frame'

# set the root folder where the images will be stored
upper_path = sep.join(args.bagfile.split(sep)[:-2])
frames_folder = os.path.join(upper_path, 'frames')
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
            img_array = img_array[:, :, [2, 1, 0]]
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
        imgs_list = sorted(glob.glob(os.path.join(
            spectralprocessor.cam_folder, '*')))
        spectralprocessor.set_white_info(white_reflectance=reflectance)
        for img_path in imgs_list:
            exif = spectralprocessor.read_exif(img_path)
            exp_t = spectralprocessor.read_exp_t_ms(img_path)
            img_array = ufunc.read_img2array(img_path)
            # compute reflectance image
            img_refl = spectralprocessor.rad_to_refl(img_array, exp_t)
            # apply spectral correction
            img_corr = spectralprocessor.corr_spectra(img_refl)
            # img_corr written as int16 (-32,768 to 32,767) to reduce size
            # keep 4 decimal digits. Reflectance 1 will correspond to 10'000
            img_corr = img_corr * 1e4
            print("Writing file {}.".format(img_path))
            ufunc.write_geotiff(img_corr, img_path, dtype=gdal.GDT_Int16)
            spectralprocessor.write_exif(img_path, exif)
