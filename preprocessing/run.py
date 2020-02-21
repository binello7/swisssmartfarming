#!../venv3/bin/python3

import os
from preprocess import Preprocessor
from PIL import Image
import utils.functions as ufunc

sep = os.path.sep
# parser = argparse.ArgumentParser(
#     description=
#         """Preprocesses an SSF-rosbag dataset.
#         |n
#         Given a rosbag file contianing a dataset for the Swiss Smart Farming
#         Project performs the preprocesses steps in order to produce
#         georeferenced corrected images that can be fed in to the Pix4D
#         software.""",
#     formatter_class=MultilineFormatter
# )
# parser.add_argument('--bag_file', '-b',
#     required=True,
#     help='Path to the bag file and name, e.g. ./dataset/Big.bag')
#
# args = parser.parse_args()


img_path = r"/media/seba/Samsung_2TB/temp/frick/20190716/rgb/frame_000027.jpg"
bagfile_auto = "/media/seba/Samsung_2TB/Matterhorn.Project/Datasets/frick/20190626/bag/2019-06-26-14-20-34.bag"
bagfile_ximea = "/media/seba/Samsung_2TB/Matterhorn.Project/Datasets/eschikon/20190527/bag/2019-05-27-14-53-54.bag"

# Create the Preprocessor object
bagfile = bagfile_ximea
preprocessor = Preprocessor(bagfile)

# Set image properties
prefix = 'frame'

# Set the root folder where the images will be stored
project_folder = sep.join(bagfile.split(sep)[:-2])
images_folder = os.path.join(project_folder, 'frames')
if not os.path.isdir(images_folder):
    os.makedirs(images_folder)

# Loop over cameras and process them
for cam in preprocessor.imgs_topics.keys():
    # Create one folder per camera
    camera_folder = os.path.join(images_folder, cam)
    if not os.path.isdir(camera_folder):
        os.makedirs(camera_folder)

    preprocessor.set_cam_info(cam)
    try:
        exp_time_topic = preprocessor.exp_time_topics[cam]
    except KeyError:
        pass
    else:
        preprocessor.set_exp_t_data(cam) #TODO: implement reading exposure time from yaml file

    msgs = preprocessor.read_img_msgs(preprocessor.imgs_topics[cam])
    for i, msg in enumerate(msgs):
        # get one image after another with its timestamp and process it
        img_tstamp = msg.timestamp.to_nsec()
        img_array = preprocessor.imgmsg_to_cv2(msg)

        # gps-rtk data were read already. Set img_info attribute
        preprocessor.set_img_info(img_tstamp)
        exp_t = None
        # if isn preprocessor.exp_t_data != None:
        #     exp_t = preprocessor.interp_exp_t(img_tstamp)

        # exif_dict = preprocessor.write_exif_dict(exp_t=exp_t)
        # exif_bytes = px.dump(exif_dict) #TODO: add number of bands to exif? think about it




        # Do different processing steps depending on the camera type
        if preprocessor.cam_info['type'] == 'RGB':
            # set filename and path
            extension = '.jpg'
            fname = prefix + '_{:05d}'.format(i)
            fname = fname + extension
            full_fname = os.path.join(camera_folder, fname)

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
            ufunc.write_geotiff(img_array, full_fname)
            preprocessor.write_exif(full_fname)
