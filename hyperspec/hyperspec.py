import spectral as sp
import numpy as np
import xml.dom.minidom as mdom
from IPython import embed
import os

sep = os.path.sep

def say_hi():
    print("hi, I am hyperspec module")

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

def read_corr_matrix(file_name):
    xml = mdom.parse(file_name)
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
