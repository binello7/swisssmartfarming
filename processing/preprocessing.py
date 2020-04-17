import argparse
import textwrap
import os
import sys
import glob
import math
from fractions import Fraction
import rosbag
import warnings
import rootpath as rp
import pandas as pd
import numpy as np
import rasterio as rio
import utils.functions as ufunc
from scipy import ndimage
from cv_bridge import CvBridge
import pyexiv2 as px2
import yaml
import xml.dom.minidom as mdom
from datetime import datetime as dt
import tkFileDialog
import Tkinter as tk
import matplotlib.pyplot as plt
from roipoly import RoiPoly
from IPython import embed


class CfgFileNotFoundError(EnvironmentError):
    pass
#===============================================================================

class NoMessagesError(Exception):
    pass
#===============================================================================

class Preprocessor(object):
    def __init__ (self, cams_cfg_path='cfg/cameras'):
        self._sep = os.path.sep
        self._rootpath = ufunc.add_sep(rp.detect())
        self.cams_cfg_path = ufunc.add_sep(cams_cfg_path)
        self.cams = None
        self.cam_info = {
            'make': None,
            'model': None,
            'type': None,
            'focal_length_mm': None,
            'img_topic': None,
            'exp_t_topic': None
        }
        self.xml_file = None
#===============================================================================

class BasePreprocessor(Preprocessor):
    def __init__(self, bagfile, rtk_topic='/ssf/dji_sdk/rtk_position',
        timezone=2):
        super(BasePreprocessor, self).__init__()
        print('Reading bagfile...')
        self.bagfile = rosbag.Bag(bagfile, 'r')
        self.bridge = CvBridge()
        self.encoding = "passthrough"
        self.img_info = {
            'date_time_orig': None,
            'subsec_time_orig': None,
            'exp_t_ms': None,
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
        self.exp_t_topics = None
        self._set_cams_and_topics()
        self.rtk_topic = rtk_topic
        self.rtk_data = None
        self._set_rtk_data()
        self.exp_t_data = pd.DataFrame(columns=["tstamp", "exp_t_ms"])
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

    def _set_cams_and_topics(self):
        cfg_paths = glob.glob(self._rootpath + self.cams_cfg_path + '*' +
            self._sep)
        if cfg_paths == []:
            raise CfgFileNotFoundError("No camera cfg folder found at the "
                "specified location '{}'. Verify 'cams_cfg_path'.".format(
                    self.cams_cfg_path))
        else:
            imgs_topics = {}
            exp_t_topics = {}
            cams = []
            for path in cfg_paths:
                cam = path.split(self._sep)[-2]
                path = os.path.join(path, (cam + '.cfg'))
                cam_cfg = self._cfg_to_dict(path)
                img_topic = cam_cfg['img_topic']
                exp_t_topic = cam_cfg['exp_t_topic']
                if img_topic in self.topics:
                    cams.append(cam)
                    imgs_topics.update({cam: img_topic})
                if exp_t_topic in self.topics:
                    exp_t_topics.update({cam: exp_t_topic})
            self.imgs_topics = imgs_topics
            self.exp_t_topics = exp_t_topics
            cams.sort()
            self.cams = cams
#-------------------------------------------------------------------------------

    def _tstamp_to_datetime_subsec(self, tstamp):
        tstamp_corr = tstamp + self._h_to_ns(self.timezone)
        dt_corr = dt.fromtimestamp(tstamp_corr / 1e9)

        return (dt_corr.strftime('%Y:%m:%d %H:%M:%S'), str(dt_corr.microsecond))
#-------------------------------------------------------------------------------

    def _msec_to_rational(self, msec):
        sec = msec / 1000
        return (1, int(round(1/sec)))
#-------------------------------------------------------------------------------

    def _latlon_to_rational(self, lat_lon, sec_precision=5):
        lat_lon = abs(lat_lon)
        deg = int(lat_lon)
        min = int((lat_lon - deg) * 60)
        sec = (lat_lon - deg - min/60) * 60**2
        sec = Fraction(int(sec * 10**sec_precision), 10**sec_precision)
        return [Fraction(deg, 1), Fraction(min, 1), sec]
#-------------------------------------------------------------------------------

    def _rational_to_str(self, rational):
        return str(rational.numerator) + '/' + str(rational.denominator)
#-------------------------------------------------------------------------------

    def _rationallist_to_str(self, rational_list):
        tmp = []
        for el in rational_list:
            tmp.append(str(el.numerator) + '/' + str(el.denominator))
        return ' '.join(tmp)
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

    def _set_exp_t_data(self):
        if self.cam_info['exp_t_topic'] in self.exp_t_topics.values():
            messages = self.bagfile.read_messages(
                topics=self.cam_info['exp_t_topic'])
            for msg in messages:
                tstamp = msg.timestamp.to_nsec()
                camera = list(self.exp_t_topics.keys())[list(
                    self.exp_t_topics.values()).index(self.cam_info['exp_t_topic'])]
                if camera == "ximea_nir":
                    exp_t = msg.message.data / 1000
                else:
                    exp_t = msg.message.exposure_time_ms
                data = {'tstamp': tstamp, 'exp_t_ms': exp_t}
                self.exp_t_data = self.exp_t_data.append(data,
                    ignore_index=True)

        else:
            warnings.warn("No topic '{}' found. Exposure time will be loaded "
                "from yaml file.".format(self.cam_info['exp_t_topic']))
            yaml_path = self._sep.join(
                self.bagfile.filename.split(self._sep)[:-1])
            yaml_basename, _ = ufunc.get_file_basename(
                self.bagfile.filename)
            yaml_file = os.path.join(yaml_path, yaml_basename) + '.yaml'

            with open(yaml_file) as file:
                try:
                    yaml_dict = yaml.safe_load(file)
                    topic_splits = self.cam_info[
                        'exp_t_topic'].split('/')
                    for s in topic_splits[1:]:
                        yaml_dict = yaml_dict[s]
                    exp_t = yaml_dict
                    tstamp = dt.strptime((self.date + " " + self.time),
                        '%Y:%m:%d %H:%M:%S').timestamp() * 1e9
                    data = {'tstamp': tstamp, 'exp_t_ms': exp_t}
                    self.exp_t_data = self.exp_t_data.append(data,
                        ignore_index=True)

                except EnvironmentError as e:
                    raise EnvironmentError(("File '{}' was not found. "
                        "Please make sure that the file is available "
                        "and has the same basename as the bagfile.").format(
                            yaml_file))
#-------------------------------------------------------------------------------

    def set_cam_info(self, cam_name):
        cfg_folder = os.path.join(self._rootpath, self.cams_cfg_path, cam_name)
        cfg_file = os.path.join(cfg_folder, '{}.cfg'.format(cam_name))
        with open(cfg_file) as file:
            cam_info = yaml.safe_load(file)

        if sorted(cam_info.keys()) == sorted(self.cam_info.keys()):
            self.cam_info.update(cam_info)
            # new camera is created. Reset all (img_info, hs_info, ...)
            self.img_info = self.img_info.fromkeys(self.img_info, None)
            self.hs_info = self.hs_info.fromkeys(self.hs_info, None)
            self.xml_file = None
            self.exp_t_data = pd.DataFrame(columns=["tstamp", "exp_t_ms"])
            if cam_info['exp_t_topic'] != None:
                self._set_exp_t_data()

            if self.cam_info['type'] == 'hyperspectral':
                xml_file = glob.glob(os.path.join(cfg_folder, '*.xml'))
                if xml_file == []:
                    warnings.warn(("No xml file found for camera '{}'. "
                        "Hyperspectral preprocessing will be skipped.").format(
                            cam_name))
                    self.xml_file = None
                else:
                    self.xml_file = xml_file[0]
                    self._set_filter_dims()
                    self._set_offsets()
                    self._set_nb_bands()

        else:
            cam_prop = ["'{}'".format(k) for k in self.cam_info.keys()]
            cam_prop = ", ".join(cam_prop)
            raise ValueError(("Wrong camera properties. Camera properties must "
                "be {}. Please correct cfg file.").format(cam_prop))
#-------------------------------------------------------------------------------

    def set_img_info(self, tstamp):
        (self.img_info['date_time_orig'], self.img_info['subsec_time_orig']) = (
            self._tstamp_to_datetime_subsec(tstamp))
        for gps_prop in self.rtk_data.keys()[1:]:
            self.img_info[gps_prop] = np.interp(tstamp,
                self.rtk_data['tstamp'], self.rtk_data[gps_prop])

        if not self.exp_t_data.empty:
            self.img_info['exp_t_ms'] = np.interp(tstamp,
                self.exp_t_data['tstamp'], self.exp_t_data['exp_t_ms'])
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
            return img
        else:
            img = img[self.hs_info['offset_y']:self.hs_info['offset_y']
                + self.hs_info['filter_h'],
                self.hs_info['offset_x']:self.hs_info['offset_x']
                + self.hs_info['filter_w']]
            pattern_len = int(math.sqrt(self.hs_info['nb_bands']))
            img_res = np.zeros((int(img.shape[0]/pattern_len),
                int(img.shape[1]/pattern_len), self.hs_info['nb_bands']))

            b = 0
            for i in range(pattern_len):
                for j in range(pattern_len):
                    img_tmp = img[np.arange(i, img.shape[0], pattern_len), :]
                    img_res[:, :, b] = img_tmp[:, np.arange(j, img.shape[1],
                        pattern_len)]
                    b+=1
            return img_res
#-------------------------------------------------------------------------------

    def median_filter_3x3(self, img):
        if self.xml_file == None:
            warnings.warn(("No xml file found. Image reshaping was skipped "
                "and median filtering cannot be applied either. Skipping "
                "3x3 median filtering."))
            return img

        elif (self.xml_file != None and len(img.shape) != 3):
            warnings.warn(('The image has shape {}, therefore it was not '
                'resampled. Apply image resampling before 3x3 median '
                'filtering. Skipping 3x3 median filtering.').format(img.shape))
            return img

        else:
            img_med = ndimage.median_filter(img, size=(3, 3, 1),
                mode='reflect')
            return img_med
#-------------------------------------------------------------------------------

    def write_exif(self, filename):
        img = px2.Image(filename)
        metadata = img.read_exif()

        # NOTE: not very elegant...
        if self.img_info['gps_lat'] > 0:
            lat_ref = 'N'
        else:
            lat_ref = 'S'
        if self.img_info['gps_lon'] > 0:
            lon_ref = 'E'
        else:
            lon_ref = 'W'
        if self.img_info['gps_alt'] > 0:
            alt_ref = "0"
        else:
            alt_ref = "1"

        meta_dict = {
            'Exif.Image.Make': self.cam_info['make'],
            'Exif.Image.Model': self.cam_info['model'],
            'Exif.Photo.FocalLength': self._rational_to_str(Fraction(self.cam_info[
                'focal_length_mm'])),
            'Exif.Photo.DateTimeOriginal': self.img_info['date_time_orig'],
            'Exif.Photo.SubSecTimeOriginal': self.img_info['subsec_time_orig'],
            'Exif.GPSInfo.GPSLatitude': self._rationallist_to_str(
                self._latlon_to_rational(self.img_info['gps_lat'])),
            'Exif.GPSInfo.GPSLatitudeRef': lat_ref,
            'Exif.GPSInfo.GPSLongitude': self._rationallist_to_str(
                self._latlon_to_rational(self.img_info['gps_lon'])),
            'Exif.GPSInfo.GPSLongitudeRef': lon_ref,
            'Exif.GPSInfo.GPSAltitude': str(Fraction(
                int(self.img_info['gps_alt']*100), 100)),
            'Exif.GPSInfo.GPSAltitudeRef': alt_ref
        }

        if self.img_info['exp_t_ms'] != None:
            meta_dict.update({'Exif.Photo.ExposureTime': str(Fraction(
                int(self.img_info['exp_t_ms']*100), 100000))})

        metadata.update(meta_dict)
        img.modify_exif(metadata)
#===============================================================================

class SpectralProcessor(Preprocessor):
    def __init__(self, frames_folder):
        super(SpectralProcessor, self).__init__()
        self.frames_folder = frames_folder
        self._set_cams()
        self.is_hyperspectral = None
        self.cam_name = None
        self.cam_folder = None
        self.white_reflectance = None
        self.white_reference_path = None
        self.white_mean_values = None
        self.white_exp_t = None
        self.corr_matrix = None
        self.virtual_wavelengths = None
        self.exif = None
#-------------------------------------------------------------------------------

    def _set_cams(self):
        cams = sorted(os.listdir(self.frames_folder))
        self.cams = cams
#-------------------------------------------------------------------------------

    def _set_hs_properties(self):
        cfg_folder = os.path.join(self._rootpath, self.cams_cfg_path,
            self.cam_name)
        cfg_file = os.path.join(cfg_folder, '{}.cfg'.format(self.cam_name))
        xml_file = glob.glob(os.path.join(cfg_folder, '*.xml'))
        if self.is_hyperspectral:
            if xml_file == []:
                raise EnvironmentError(("No xml file found for camera '{}'. "
                    "Hyperspectral preprocessing not possible.").format(
                        self.cam_name))
            else:
                self.xml_file = xml_file[0]
                self._set_corr_matrix_and_wavelengths()
        else:
            self.xml_file = None
            self.virtual_wavelengths = None
            self.corr_matrix = None
#-------------------------------------------------------------------------------

    def _set_corr_matrix_and_wavelengths(self):
        xml = mdom.parse(self.xml_file)
        virtual_bands = xml.getElementsByTagName("virtual_band")
        wavelengths = []
        coefficients = []

        for virtual_band in virtual_bands:
            wavelength = float(str(virtual_band.childNodes.item(1)
                .firstChild).split("'")[1])
            wavelengths.append(wavelength)
            coeffs = str(virtual_band.childNodes.item(5)
                .firstChild.data).split(', ')

            data = []
            for coeff in coeffs:
                coeff = float(coeff)
                data.append(coeff)

            coefficients.append(data)

        self.corr_matrix = np.array(coefficients)
        self.virtual_wavelengths = np.array(wavelengths)
#-------------------------------------------------------------------------------

    def read_exif(self, img_path):
        img = px2.Image(img_path)
        exif = img.read_exif()
        return [exif, img]
#-------------------------------------------------------------------------------

    def read_exp_t_ms(self, img_path):
        exif = self.read_exif(img_path)[0]
        exp_t = Fraction(str(exif['Exif.Photo.ExposureTime']))
        exp_t = float(exp_t) * 1e3
        return exp_t
#-------------------------------------------------------------------------------

    def write_exif(self, img_path, img, exif):
        exif_new, img_new = self.read_exif(img_path)
        for exif_key in exif_new:
            exif_val = str(exif_new[exif_key])
            exif.update({exif_key: exif_val})
        img_new.modify_exif(exif)
#-------------------------------------------------------------------------------

    def set_cam_info(self, cam_name):
        self.cam_name = cam_name
        self.cam_folder = os.path.join(self.frames_folder, cam_name)
        cfg_folder = os.path.join(self._rootpath, self.cams_cfg_path, cam_name)
        cfg_file = os.path.join(cfg_folder, '{}.cfg'.format(cam_name))
        with open(cfg_file) as file:
            cam_info = yaml.safe_load(file)
        if sorted(cam_info.keys()) == sorted(self.cam_info.keys()):
            self.cam_info.update(cam_info)
            if cam_info['type'] == 'hyperspectral':
                self.is_hyperspectral = True
            else:
                self.is_hyperspectral = False
            self._set_hs_properties()

        else:
            cam_prop = ["'{}'".format(k) for k in self.cam_info.keys()]
            cam_prop = ", ".join(cam_prop)
            raise ValueError(("Wrong camera properties. Camera properties must "
                "be '{}'. Please correct cfg file.").format(cam_prop))
#-------------------------------------------------------------------------------

    def set_white_info(self, white_reflectance):
        self.white_reflectance = white_reflectance
        root = tk.Tk()
        root.withdraw()
        white_reference_path =  tkFileDialog.askopenfilename(
            initialdir=self.cam_folder,
            title="Select white reference image for '{}' camera".format(
                self.cam_name))
        root.destroy()
        self.white_reference_path = white_reference_path
        white_exp_t = self.read_exp_t_ms(white_reference_path)
        self.white_exp_t = white_exp_t

        white_array = ufunc.read_img2array(white_reference_path)
        bands = white_array.shape[2]
        band = int(bands / 2)

        fig = plt.figure()
        plt.imshow(white_array[:,:,band], cmap=plt.get_cmap("Greys_r"))
        plt.show(block=False)

        roi = RoiPoly(color='r', fig=fig)

        mask = roi.get_mask(white_array[:,:,band])
        white_mean_values = np.mean(white_array[mask], axis=0)
        self.white_mean_values = white_mean_values
#-------------------------------------------------------------------------------

    def rad_to_refl(self, img_array, img_exp_t):
        img_refl = ((img_array / self.white_mean_values) * (self.white_exp_t /
            img_exp_t) * self.white_reflectance)
        max_refl = np.max(img_refl)
        if max_refl > 1:
            warnings.warn(('Attention: max reflectance > 1.0: max_refl={:.2f}'
                .format(max_refl)))
        return img_refl
#-------------------------------------------------------------------------------

    def corr_spectra(self, img_refl):
        # error if correction matrix not loaded
        rows = img_refl.shape[0]
        cols = img_refl.shape[1]
        img_corr = np.zeros((rows, cols, self.corr_matrix.shape[0]))
        for i in range(rows):
            for j in range(cols):
                img_corr[i,j,:] = np.dot(self.corr_matrix, img_refl[i,j,:])

        # reflectance should be within [0,1]. Perform checks
        max_corr_refl = np.max(img_corr)
        min_corr_refl = np.min(img_corr)
        if min_corr_refl < 0:
            warnings.warn(("Attention: corrected min reflectance < 0 detected: "
                "min_corr_refl={:.2f}").format(min_corr_refl))
        if max_corr_refl > 1:
            warnings.warn(("Attention: corrected max reflectance > 1 detected: "
                "max_corr_refl={:.2f}").format(max_corr_refl))
        return img_corr
