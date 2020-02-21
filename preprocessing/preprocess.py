#!../venv/bin/python3

import argparse
import textwrap
import os
import sys
import glob
import math
import rosbag
import warnings
import rootpath as rp
import pandas as pd
import numpy as np
import rasterio as rio
import utils.functions as ufunc
from cv_bridge import CvBridge
import piexif as px
import pyexiv2
import yaml
import xml.dom.minidom as mdom
from datetime import datetime as dt
from PIL import Image
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

class CfgFileNotFoundError(FileNotFoundError):
    pass
#===============================================================================

class NoMessagesError(Exception):
    pass
#===============================================================================

class Preprocessor:
    def __init__(self, bagfile, rtk_topic='/ssf/dji_sdk/rtk_position',
        cam_cfg_path='cfg/cameras', timezone=2,
            exp_time_topics = {
                'ximea_nir':
                    '/ximea_asl/exposure_time',
                'photonfocus_nir':
                    '/ssf/photonfocus_camera_nir_node/exposure_time_ms',
                'photonfocus_vis':
                    '/ssf/photonfocus_camera_vis_node/exposure_time_ms'
            }
    ):
        self._sep = os.path.sep
        self._rootpath = ufunc.add_sep(rp.detect())
        self.bagfile = rosbag.Bag(bagfile, 'r')
        self.bridge = CvBridge()
        self.encoding = "passthrough"
        self.cam_cfg_path = ufunc.add_sep(cam_cfg_path)
        self.xml_file = None
        self.cam_info = {
            'make': None,
            'model': None,
            'type': None,
            'focal_length_mm': None,
            'img_topic': None
        }
        self.img_info = {
            'date_time_orig': None,
            'gps_lat': None,
            'gps_lon': None,
            'gps_alt': None
        }
        self.hs_info = {
            'filter_w': None,
            'filter_h': None,
            'offset_x': None,
            'offset_y': None,
            'nb_bands': None
        }
        topics = self.bagfile.get_type_and_topic_info().topics.keys()
        topics = [t for t in topics]
        self.topics = topics
        self.imgs_topics = None
        self.exp_time_topics = exp_time_topics
        self.cams = None
        self._set_cams_and_imgs_topics()
        self.rtk_topic = rtk_topic
        self.rtk_data = None
        self._set_rtk_data()
        self.exp_t_data = None
        self.date = self._read_date_time()[0]
        self.time = self._read_date_time()[1]
        self.timezone = timezone
#-------------------------------------------------------------------------------

    def _read_date_time(self):
        filename = self.bagfile.filename.split(self._sep)[-1]
        filename = filename.split('.')[0]
        date = ":".join(filename.split('-')[:3])
        time = ":".join(filename.split('-')[3:])
        return (date, time)
#-------------------------------------------------------------------------------

    def _h_to_ns(self, hours):
        ns = hours * 60 * 60 * 10^9
        return ns
#-------------------------------------------------------------------------------

    def _cfg_to_dict(self, cfg_file):
        with open(cfg_file) as file:
            cam_cfg = yaml.safe_load(file)
        return cam_cfg
#-------------------------------------------------------------------------------

    def _set_cams_and_imgs_topics(self):
        cfg_paths = glob.glob(self._rootpath + self.cam_cfg_path + '*' +
            self._sep)
        if cfg_paths == []:
            raise CfgFileNotFoundError("No camera cfg folder found at the "
                "specified location '{}'. Verify 'cam_cfg_path'.".format(
                    self.cam_cfg_path))
        else:
            imgs_topics = {}
            cams = []
            for path in cfg_paths:
                cam = path.split(self._sep)[-2]
                path = os.path.join(path, (cam + '.cfg'))
                cam_cfg = self._cfg_to_dict(path)
                img_topic = cam_cfg['img_topic']
                if img_topic in self.topics:
                    cams.append(cam)
                    topic_dict = {cam: img_topic}
                    imgs_topics.update(topic_dict)
            self.imgs_topics = imgs_topics
            self.cams = cams
#-------------------------------------------------------------------------------

    def _tstamp_to_date_time_orig(self, tstamp):
        tstamp_corr = tstamp + self._h_to_ns(self.timezone)
        dt_corr = dt.fromtimestamp(tstamp_corr / 1e9)
        return dt_corr.strftime('%Y:%m:%d %H:%M:%S')
#-------------------------------------------------------------------------------

    def _msec_to_rational(self, msec):
        sec = msec / 1000
        return (1, int(round(1/sec)))
#-------------------------------------------------------------------------------

    def _float_to_rational(self, val_float, precision=1000):
        val_rat = int(val_float * precision)
        return (val_rat, precision)
#-------------------------------------------------------------------------------

    def _latlon_to_rational(self, lat_lon, a=1e7):
        lat_lon = abs(lat_lon)
        deg = int(lat_lon)
        min = int((lat_lon - deg) * 60)
        sec = (lat_lon - deg - min/60) * 60**2
        sec = int(sec * a)
        return ((deg, 1), (min, 1), (sec, int(a)))
#-------------------------------------------------------------------------------

    def _set_rtk_data(self):
        messages = self.bagfile.read_messages(topics=self.rtk_topic)
        try:
            next(messages)
        except StopIteration as e:
            raise NoMessagesError("No RTK-GPS messages found. Check the RTK "
                "topic '{}' for correctness and verify if the topic is not "
                "empty.".format(self.rtk_topic))
        else:
            rtk_data = pd.DataFrame(columns=["tstamp", "gps_lat", "gps_lon",
                "gps_alt"])
            for msg in messages:
                tstamp = msg.timestamp.to_nsec()
                lat = msg.message.latitude
                lon = msg.message.longitude
                alt = msg.message.altitude
                data = pd.Series(data={'tstamp': tstamp, 'gps_lat': lat,
                    'gps_lon': lon, 'gps_alt': alt})
                rtk_data = rtk_data.append(data, ignore_index=True)
            self.rtk_data = rtk_data
#-------------------------------------------------------------------------------

    def _set_filter_dims(self):
        xml = mdom.parse(self.xml_file)
        height = int(str(xml.getElementsByTagName("height")[1].firstChild.data))
        width = int(str(xml.getElementsByTagName("width")[1].firstChild.data))
        self.hs_info['filter_h'] = height
        self.hs_info['filter_w'] = width
#-------------------------------------------------------------------------------

    def _set_offsets(self):
        xml = mdom.parse(self.xml_file)
        offset_x = xml.getElementsByTagName("offset_x")
        offset_y = xml.getElementsByTagName("offset_y")
        offset_x = int(str(offset_x.item(0).firstChild.data))
        offset_y = int(str(offset_y.item(0).firstChild.data))
        self.hs_info['offset_x'] = offset_x
        self.hs_info['offset_y'] = offset_y
#-------------------------------------------------------------------------------

    def _set_nb_bands(self):
        xml = mdom.parse(self.xml_file)
        bands_width = xml.getElementsByTagName("pattern_width")
        bands_height = xml.getElementsByTagName("pattern_height")
        bands_width = int(str(bands_width.item(0).firstChild.data))
        bands_height = int(str(bands_height.item(0).firstChild.data))
        self.hs_info['nb_bands'] = bands_height * bands_width
#-------------------------------------------------------------------------------

    def interp_exp_t(self, tstamp):
        exp_t = np.interp(tstamp, self.exp_t_data['tstamp'],
            self.exp_t_data['exp_t_ms'])
        return exp_t
#-------------------------------------------------------------------------------

    def set_exp_t_data(self, camera):
        if self.exp_time_topics[camera] in self.topics:
            messages = self.bagfile.read_messages(topics=
                self.exp_time_topics[camera])
            exp_t_data = pd.DataFrame(columns=["tstamp", "exp_t_ms"])
            for msg in messages:
                tstamp = msg.timestamp.to_nsec()
                if camera == "ximea_nir":
                    exp_t = msg.message.data / 1000
                else:
                    exp_t = msg.message.exposure_time_ms
                data = pd.Series(data={'tstamp': tstamp, 'exp_t_ms': exp_t})
                exp_t_data = exp_t_data.append(data, ignore_index=True)
            self.exp_t_data = exp_t_data
        else:
            warnings.warn("No topic '{}' found. Exposure time will be loaded "
                "from yaml file.".format(self.exp_time_topics[camera]))
            self.exp_t_data = None
#-------------------------------------------------------------------------------

    def set_cam_info(self, camera):
        cfg_folder = os.path.join(self._rootpath, self.cam_cfg_path, camera)
        cfg_file = os.path.join(cfg_folder, '{}.cfg'.format(camera))
        with open(cfg_file) as file:
            cam_info = yaml.safe_load(file)
        if cam_info.keys() == self.cam_info.keys():
            self.cam_info.update(cam_info)
            if self.cam_info['type'] == 'hyperspectral':
                xml_file = glob.glob(os.path.join(cfg_folder, '*.xml'))
                if xml_file == []:
                    warnings.warn(("No xml file found for camera '{}'. "
                        "Hyperspectral preprocessing will be skipped.").format(
                            camera))
                    self.xml_file = None
                else:
                    self.xml_file = xml_file[0]
                    self._set_filter_dims()
                    self._set_offsets()
                    self._set_nb_bands()
            else:
                self.xml_file = None
                self.hs_info = self.hs_info.fromkeys(self.hs_info, None)
        else:
            cam_prop = ["'{}'".format(k) for k in self.cam_info.keys()]
            cam_prop = ", ".join(cam_prop)
            raise ValueError(("Wrong camera properties. Camera properties must "
                "be {}. Please correct cfg file.").format(cam_prop))
#-------------------------------------------------------------------------------

    def set_img_info(self, tstamp):
        self.img_info['date_time_orig'] = self._tstamp_to_date_time_orig(tstamp)
        for gps_prop in self.rtk_data.keys()[1:]:
            self.img_info[gps_prop] = np.interp(tstamp,
                self.rtk_data['tstamp'], self.rtk_data[gps_prop])
#-------------------------------------------------------------------------------

    def read_img_msgs(self, imgs_topic):
        return self.bagfile.read_messages(topics=imgs_topic)
#-------------------------------------------------------------------------------

    def imgmsg_to_cv2(self, message):
        return self.bridge.imgmsg_to_cv2(message.message,
            desired_encoding=self.encoding)
#-------------------------------------------------------------------------------

    def reshape_hs(self, img):
        if self.xml_file == None:
            warnings.warn("No xml file found. Skipping image reshaping.")
            return
        else:
            img = img[self.hs_info['offset_y']:self.hs_info['offset_y']
                + self.hs_info['filter_h'],
                self.hs_info['offset_x']:self.hs_info['offset_x']
                + self.hs_info['filter_w']]
            pattern_len = int(math.sqrt(self.hs_info['nb_bands']))
            img_res = np.zeros((int(img.shape[0]/pattern_len),
                int(img.shape[1]/pattern_len), self.hs_info['nb_bands']))

            for b, i in enumerate(range(pattern_len)):
                for j in range(pattern_len):
                    img_tmp = img[np.arange(i, img.shape[0], pattern_len), :]
                    img_res[:, :, b] = img_tmp[:, np.arange(j, img.shape[1],
                        pattern_len)]
            return img_res
#-------------------------------------------------------------------------------

    def write_exif_dict(self, exp_t=None):
        val_cam_info = self.cam_info.values()
        val_cam_info = [val for val in val_cam_info]
        val_cam_info = np.array(val_cam_info)
        if any(val_cam_info==None):
            raise ValueError(("'None' values detected. Set attribute "
                "'cam_info' before writing the exif-dictionary."))
        else:
            zeroth_ifd = {
                px.ImageIFD.Make: self.cam_info['make'],
                px.ImageIFD.Model: self.cam_info['model']
            }
            exif_ifd = {
                px.ExifIFD.FocalLength: self._float_to_rational(
                    self.cam_info['focal_length_mm'], precision=100),
                px.ExifIFD.DateTimeOriginal: self.img_info['date_time_orig']
            }
            if exp_t != None:
                exif_ifd.update({px.ExifIFD.ExposureTime:
                    self._msec_to_rational(exp_t)})

            if self.img_info['gps_lat'] > 0:
                lat_ref = 'N'
            else:
                lat_ref = 'S'

            if self.img_info['gps_lon'] > 0:
                lon_ref = 'E'
            else:
                lon_ref = 'W'

            if self.img_info['gps_alt'] > 0:
                alt_ref = 0
            else:
                alt_ref = 1

            gps_ifd = {
                px.GPSIFD.GPSLatitude: self._latlon_to_rational(
                    self.img_info['gps_lat']),
                px.GPSIFD.GPSLatitudeRef: lat_ref,
                px.GPSIFD.GPSLongitude: self._latlon_to_rational(
                    self.img_info['gps_lon']),
                px.GPSIFD.GPSLongitudeRef: lon_ref,
                px.GPSIFD.GPSAltitude: self._float_to_rational(
                    self.img_info['gps_alt']),
                px.GPSIFD.GPSAltitudeRef: alt_ref
            }
            first_ifd = {}
            exif_dict = {"0th":zeroth_ifd, "Exif":exif_ifd, "GPS":gps_ifd,
                "1st":first_ifd}
        return exif_dict
#===============================================================================

if __name__ == "__main__":
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
    images_folder = os.path.join(project_folder, 'images')
    if not os.path.isdir(images_folder):
        os.makedirs(images_folder)

    # Loop over cameras and process them
    for cam in preprocessor.imgs_topics.keys():
        # Create one folder per camera
        camera_folder = os.path.join(images_folder, cam)
        if not os.path.isdir(camera_folder):
            os.makedirs(camera_folder)

        preprocessor.set_cam_info(cam)
        # embed()
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

            exif_dict = preprocessor.write_exif_dict(exp_t=exp_t)
            exif_bytes = px.dump(exif_dict) #TODO: add number of bands to exif? think about it




            # Do different processing steps depending on the camera type
            if preprocessor.cam_info['type'] == 'RGB':
                extension = '.jpg'
                fname = prefix + '_{:05d}'.format(i)
                fname = fname + extension
                full_fname = os.path.join(camera_folder, fname)

            elif preprocessor.cam_info['type'] == 'hyperspectral':
                # set filename and path
                extension = '.tif'
                fname = prefix + '_{:05d}'.format(i)
                fname = fname + extension
                full_fname = os.path.join(camera_folder, fname)

                # reshape the raw sensor data
                img_array = preprocessor.reshape_hs(img_array)
                ufunc.write_geotiff(img_array, full_fname)
                px.insert(exif_bytes, full_fname)

            else:
                extension = '.jpg'


            # im = Image.fromarray(img_array)
            # # im.save(full_fname, exif=exif_bytes, quality=100)
