#!../venv/bin/python3

import argparse
import textwrap
import os
import sys
import glob
import rosbag
import rootpath as rp
import pandas as pd
import numpy as np
import utils.functions as ufunc
from cv_bridge import CvBridge
import piexif as px
import yaml
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

class Preprocessor:
    def __init__(self, bagfile, rtk_topic='/ssf/dji_sdk/rtk_position',
        cam_cfg_path='cfg/cameras', timezone=2):
        self._sep = os.path.sep
        self._rootpath = ufunc.add_sep(rp.detect())
        self.bagfile = rosbag.Bag(bagfile, 'r')
        self.bridge = CvBridge()
        self.encoding = "passthrough"
        self.cam_cfg_path = ufunc.add_sep(cam_cfg_path)
        self.cam_info = {
            'make': None,
            'model': None,
            'focal_length_mm': None,
            'img_topic': None,
            'exp_t_topic': None
        }
        self.img_info = {
            'date_time_orig': None,
            'exp_time_s': None,
            'gps_lat': None,
            'gps_lon': None,
            'gps_alt': None
        }
        topics = self.bagfile.get_type_and_topic_info().topics.keys()
        topics = [t for t in topics]
        self.topics = topics
        self.imgs_topics = None
        self._set_imgs_topics()
        self.cams = None
        self._set_cams()
        self.rtk_topic = rtk_topic
        self.rtk_data = None
        self._set_rtk_data()
        self.date = self._read_date_time()[0]
        self.time = self._read_date_time()[1]
        self.timezone = timezone
        embed()
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

    def _set_imgs_topics(self):
        cfg_paths = glob.glob(self._rootpath + self.cam_cfg_path + '*.cfg')
        imgs_topics = []
        for path in cfg_paths:
            cam_cfg = self._cfg_to_dict(path)
            img_topic = cam_cfg['img_topic']
            if img_topic in self.topics:
                imgs_topics.append(img_topic)
        self.imgs_topics = imgs_topics
#-------------------------------------------------------------------------------

    def _set_cams(self):
        cfg_paths = glob.glob(self._rootpath + self.cam_cfg_path + '*.cfg')
        cams_names = []
        for path in cfg_paths:
            cam_cfg = self._cfg_to_dict(path)
            if cam_cfg['img_topic'] in self.imgs_topics:
                cam_name = ufunc.get_file_basename(path)[0]
                cams_names.append(cam_name)
        self.cams = cams_names
#-------------------------------------------------------------------------------

    def tstamp_to_date_time_orig(self, tstamp):
        tstamp_corr = tstamp + self._h_to_ns(self.timezone)
        dt_corr = dt.fromtimestamp(tstamp_corr / 1e9)
        return {'date_time_orig': dt_corr.strftime('%Y:%m:%d %H:%M:%S')}
#-------------------------------------------------------------------------------

    def _sec_to_rational(self, sec):
        return (1, int(round(1/sec)))
#-------------------------------------------------------------------------------

    def _set_rtk_data(self):
        messages = self.bagfile.read_messages(topics=self.rtk_topic)
        rtk_data = pd.DataFrame(columns=["tstamp", "lat", "lon", "alt"])
        for msg in messages:
            tstamp = msg.timestamp.to_nsec()
            lat = msg.message.latitude
            lon = msg.message.longitude
            alt = msg.message.altitude
            data = pd.Series(data={'tstamp': tstamp, 'lat': lat, 'lon': lon,
                'alt': alt})
            rtk_data = rtk_data.append(data, ignore_index=True)
        self.rtk_data = rtk_data
#-------------------------------------------------------------------------------

    def get_tstamps(self, topic):
        messages = self.bagfile.read_messages(topics=topic)
        tstamps = []
        for msg in messages:
            tstamps.append(msg.timestamp.to_nsec())
        return tstamps
#-------------------------------------------------------------------------------

    def set_cam_info(self, cfg_file):
        with open(cfg_file) as file:
            cam_info = yaml.safe_load(file)
        if cam_info.keys() == self.cam_info.keys():
            self.cam_info.update(cam_info)
        else:
            cam_prop = ["'{}'".format(k) for k in self.cam_info.keys()]
            cam_prop = ", ".join(cam_prop)
            raise ValueError(("Wrong camera properties. Camera properties must "
                "be {}.").format(cam_prop))
#-------------------------------------------------------------------------------

    def set_img_info(self, img_info):
        if img_info.keys() == self.img_info.keys():
            self.img_info.update(img_info)
        else:
            img_prop = ["'{}'".format(k) for k in self.img_info.keys()]
            img_prop = ", ".join(img_prop)
            raise ValueError(("Wrong image properties. Image properties must "
                "be {}.").format(img_prop))
#-------------------------------------------------------------------------------

    def read_img_msgs(self, imgs_topic):
        return self.bagfile.read_messages(topics=imgs_topic)
#-------------------------------------------------------------------------------

    def imgmsg_to_cv2(self, message):
        return self.bridge.imgmsg_to_cv2(message.message,
            desired_encoding=self.encoding)
#-------------------------------------------------------------------------------

    def interp_rtk_data(self, rtk_data, imgs_tstamps):
        rtk_data_interp = pd.DataFrame(
            data=np.zeros((len(imgs_tstamps), len(rtk_data.columns))),
            columns=rtk_data.columns)
        rtk_data_interp.values[:,0] = imgs_tstamps
        for col in rtk_data_interp.columns:
            rtk_data_interp[col] = np.interp(rtk_data_interp['tstamp'],
                rtk_data['tstamp'], rtk_data[col])
        return rtk_data_interp
#-------------------------------------------------------------------------------

    def write_exif_dict(self):
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
                px.ExifIFD.FocalLength: (self.cam_info['focal_length_mm'], 1),
                px.ExifIFD.ExposureTime: self._sec_to_rational(
                    self.img_info['exp_time_s']),
                px.ExifIFD.DateTimeOriginal: self.img_info['date_time_orig']
            }

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
                px.GPSIFD.GPSLatitude: self.img_info['gps_lat'],
                px.GPSIFD.GPSLatitudeRef: lat_ref,
                px.GPSIFD.GPSLongitude: self.img_info['gps_lon'],
                px.GPSIFD.GPSLongitudeRef: lon_ref,
                px.GPSIFD.GPSAltitude: self.img_info['gps_alt'],
                px.GPSIFD.GPSAltitudeRef: alt_ref
            }
            first_ifd = {}
            exif_dict = {"0th":zeroth_ifd, "Exif":exif_ifd, "GPS":gps_ifd,
                "1st":first_ifd}
        return exif_dict
#===============================================================================

if __name__ == "__main__":
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
    cfg_file = r"/home/seba/Projects/swisssmartfarming/cfg/cameras/photonfocus_nir.cfg"
    bagfile = "/media/seba/Samsung_2TB/Matterhorn.Project/Datasets/frick/2019-06-07-15-57-31.bag"

    # Create the Preprocessor object
    preprocessor = Preprocessor(bagfile)

    # Process first camera set the cam_info properties
    preprocessor.set_cam_info(cfg_file)
    imgs_tstamps = []
    for topic in imgs_topics:
        tstamps = preprocessor.get_tstamps(topic)
        imgs_tstamps.append(tstamps)

    # rtk_data_interp = preprocessor.interp_rtk_data(rtk_data, imgs_tstamps[0])
    #
    # messages = preprocessor.read_img_msgs(imgs_topics[0])
    # bridge = CvBridge()
    # img_info = {}
    # # for msg in messages:
    # msg = next(messages)
    #
    # cv2_img = preprocessor.imgmsg_to_cv2(msg)
    # tstamp = msg.timestamp.to_nsec()
    # img_info.update(preprocessor.tstamp_to_date_time_orig(tstamp))

    embed()

    #
    # exif_bytes = write_exif_dict(cfg_file)
    # im = Image.open(img_path)
    # im.save("out.jpg", exif=exif_bytes)
