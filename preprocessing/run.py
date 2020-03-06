#!../venv3/bin/python3

import os
from preprocess import Preprocessor
from PIL import Image
import utils.functions as ufunc
import argparse
import textwrap
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

sep = os.path.sep

parser = argparse.ArgumentParser(
    description=
        """Preprocesses an SSF-rosbag dataset.
        |n
        Given a rosbag file contianing a dataset for the Swiss Smart Farming
        Project performs the preprocesses steps in order to produce
        georeferenced corrected images that can be fed into the Pix4D
        software.""",
    formatter_class=MultilineFormatter
)
parser.add_argument('--bagfile', '-b',
    required=True,
    help='Path to the bag file and name, e.g. ./dataset/bagfile.bag')

args = parser.parse_args()

# Create the Preprocessor object
preprocessor = Preprocessor(args.bagfile)

# Set image properties
prefix = 'frame'

# Set the root folder where the images will be stored
project_folder = sep.join(args.bagfile.split(sep)[:-2])
images_folder = os.path.join(project_folder, 'frames')
if not os.path.isdir(images_folder):
    os.makedirs(images_folder)

# Loop over cameras and process them
for cam in preprocessor.cams:
    # Create one folder per camera
    print("Processing camera '{}'".format(cam))
    camera_folder = os.path.join(images_folder, cam)
    if not os.path.isdir(camera_folder):
        os.makedirs(camera_folder)

    preprocessor.set_cam_info(cam)
    embed()

    msgs = preprocessor.read_img_msgs(preprocessor.imgs_topics[cam])
    for i, msg in enumerate(msgs):
        # get one image after another with its timestamp and process it
        img_tstamp = msg.timestamp.to_nsec()
        img_array = preprocessor.imgmsg_to_cv2(msg)

        # gps-rtk data are read automatically. Set img_info attribute
        preprocessor.set_img_info(img_tstamp)

        # Do different processing steps depending on the camera type
        if preprocessor.cam_info['type'] == 'RGB':
            # set filename and path
            extension = '.jpg'
            fname = prefix + '_{:05d}'.format(i)
            fname = fname + extension
            full_fname = os.path.join(camera_folder, fname)
            # save image and write exif metadata
            im = Image.fromarray(img_array)
            im.save(full_fname, quality=100)
            preprocessor.write_exif(full_fname)

        elif preprocessor.cam_info['type'] == 'hyperspectral':
            # set filename and path
            extension = '.tif'
            fname = prefix + '_{:05d}'.format(i)
            fname = fname + extension
            full_fname = os.path.join(camera_folder, fname)
            # reshape the raw sensor data
            img_array = preprocessor.reshape_hs(img_array)
            # apply median filtering
            img_array = preprocessor.median_filter_3x3(img_array)
            # save image and write exif metadata
            ufunc.write_geotiff(img_array, full_fname)
            preprocessor.write_exif(full_fname)

        elif preprocessor.cam_info['type'] == 'thermal':
            # set filename and path
            extension = '.tif'
            fname = prefix + '_{:05d}'.format(i)
            fname = fname + extension
            full_fname = os.path.join(camera_folder, fname)
            # save image and write exif metadata
            im = Image.fromarray(img_array)
            im.save(full_fname)
            preprocessor.write_exif(full_fname)
