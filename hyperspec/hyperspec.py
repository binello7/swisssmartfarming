from __future__ import division
import spectral as sp
from scipy import ndimage
import numpy as np
import xml.dom.minidom as mdom
from IPython import embed
import os
import rootpath
import glob
import math

sep = os.path.sep

def read_bsq(file_path):
    path = sep.join(file_path.split(sep)[0:-1])
    hdr = file_path.split(sep)[-1].split('.')[0] + '.hdr'
    return sp.open_image(path + sep + hdr).load()
#-------------------------------------------------------------------------------

def read_raw(file_path):
    dims = (1088, 2048)
    img = np.empty(dims, np.uint16)
    img.data[:] = open(file_path).read()
    return img
#-------------------------------------------------------------------------------

def median_filter(img):
    median_filter = ndimage.median_filter(img, size=3, mode='constant')
    return median_filter
#-------------------------------------------------------------------------------

def read_corr_matrix(file_path):
    xml = mdom.parse(file_path)
    virtual_bands = xml.getElementsByTagName("virtual_band")
    wavelengths = []
    coefficients = []

    for virtual_band in virtual_bands:
        wavelength = float(str(virtual_band.childNodes.item(1).firstChild)
            .split("'")[1])
        wavelengths.append(wavelength)
        coeffs = str(virtual_band.childNodes.item(5)
            .firstChild.data).split(', ')

        data = []
        for coeff in coeffs:
            coeff = float(coeff)
            data.append(coeff)

        coefficients.append(data)

    coefficients = np.array(coefficients)
    wavelengths = np.array(wavelengths)

    return wavelengths, coefficients
#-------------------------------------------------------------------------------

def read_filter_dims(file_path):
    xml = mdom.parse(file_path)
    height = int(str(xml.getElementsByTagName("height")[1].firstChild.data))
    width = int(str(xml.getElementsByTagName("width")[1].firstChild.data))
    return (height, width)
#-------------------------------------------------------------------------------

def read_offsets(file_path):
    xml = mdom.parse(file_path)
    offset_x = xml.getElementsByTagName("offset_x")
    offset_y = xml.getElementsByTagName("offset_y")
    offset_x = int(str(offset_x.item(0).firstChild.data))
    offset_y = int(str(offset_y.item(0).firstChild.data))
    return (offset_x, offset_y)
#-------------------------------------------------------------------------------

def read_nb_bands(file_path):
    xml = mdom.parse(file_path)
    bands_width = xml.getElementsByTagName("pattern_width")
    bands_height = xml.getElementsByTagName("pattern_height")
    bands_width = int(str(bands_width.item(0).firstChild.data))
    bands_height = int(str(bands_height.item(0).firstChild.data))
    return bands_height * bands_width
#-------------------------------------------------------------------------------

class HypSpecImage:
    def __init__(self, camera, raw):
        self.camera = camera
        self.raw = raw
        self.ref = None
        self.res = None
        self.med = None
        self.xml_file = glob.glob(os.path.join(rootpath.detect(), 'cfg',
            'cameras', camera, '*.xml'))[0]
        self.filter_dims = read_filter_dims(self.xml_file)
        self.offset_x = read_offsets(self.xml_file)[0]
        self.offset_y = read_offsets(self.xml_file)[1]
        self.nb_bands = read_nb_bands(self.xml_file)
#-------------------------------------------------------------------------------

    def rad_to_refl(self, white, dark_ti, dark_tw, ti, tw):
        self.ref = (self.raw - dark_ti) / (white - dark_tw) * tw / ti
#-------------------------------------------------------------------------------

    def reshape(self):
        img = self.ref
        img = img[self.offset_y:self.offset_y+self.filter_dims[0],
            self.offset_x:self.offset_x+self.filter_dims[1]]
        pattern_len = int(math.sqrt(self.nb_bands))
        img_res = np.zeros((int(img.shape[0]/pattern_len),
            int(img.shape[1]/pattern_len), self.nb_bands))

        band = 0
        for i in range(pattern_len):
            for j in range(pattern_len):
                img_tmp = img[np.arange(i, img.shape[0], pattern_len), :]
                img_res[:, :, band] = img_tmp[:, np.arange(j, img.shape[1],
                    pattern_len)]
                band += 1
        self.res = img_res
#-------------------------------------------------------------------------------

    def median_filter(self):
        median_filter = []
        for i in range(self.res.shape[2]):
            img = self.res[:, :, i]
            median_filter.append(ndimage.median_filter(img, size=3, mode='constant'))
        median_filter = np.asarray(median_filter)
        median_filter = np.moveaxis(median_filter, 0, 2)
        self.med = median_filter
