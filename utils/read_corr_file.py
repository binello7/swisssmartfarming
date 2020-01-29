#!../venv/bin/python2

import xml.dom.minidom as mdom
import numpy as np
import matplotlib.pyplot as plt
from IPython import embed

def read_responses(file_name):
    xml = mdom.parse(file_name)
    responses = xml.getElementsByTagName("response")
    wavelength_start = xml.getElementsByTagName("wavelength_range_start_nm")
    wavelength_end = xml.getElementsByTagName("wavelength_range_end_nm")
    wavelength_resolution = xml.getElementsByTagName("wavelength_resolution_nm")

    for start, end, res in zip(wavelength_start, wavelength_end, wavelength_resolution):
        start = float(str(start.childNodes.item(0).data))
        end = float(str(end.childNodes.item(0).data))
        res = float(str(res.childNodes.item(0).data))

    data = []
    for response in responses:
        response = response.childNodes.item(0)
        response = response.data
        response = str(response)
        response = response.split(', ')
        vals = []
        for val in response:
            vals.append(float(val))

        data.append(vals)

    data = (np.array(data))
    responses = data[0:-2,:]
    coeffs = data[-2:,:]
    wavelengths = np.arange(start, end+res, res)
    return responses, coeffs, wavelengths

def read_peaks(file_name):
    xml = mdom.parse(file_name)
    peaks = xml.getElementsByTagName("peak")
    peaks_wavelengths = []
    peaks_contributions = []

    for peak in peaks:
        wavelenght = float(str(peak.childNodes.item(1).firstChild).split("'")[1])
        peaks_wavelengths.append(wavelenght)
        contribution = float(str(peak.childNodes.item(7).firstChild).split("'")[1])
        peaks_contributions.append(contribution)

    peaks_wavelengths = np.array(peaks_wavelengths)
    peaks_contributions = np.array(peaks_contributions)

    return peaks_wavelengths, peaks_contributions

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



if __name__ == "__main__":
    xml_vis = "/home/seba/polybox/Matterhorn.Project/Sensors/Photonfocus/CMV2K-SSM4x4-470_620-9.4.10.10_HS03-VIS.xml"
    xml_nir = "/home/seba/polybox/Matterhorn.Project/Sensors/Photonfocus/CMV2K-SSM5x5-665_975-13.8.15.1_HS02-NIR.xml"
    responses, coeffs, wavelengths = read_responses(xml_vis)
    responses = responses * coeffs[0] * coeffs[1]
    responses = np.transpose(responses)

    peaks, contributions = read_peaks(xml_vis)

    bands_idx = np.arange(0, responses.shape[1]+1)
    legend = []
    for idx in bands_idx:
        band_name = "band_" + str(idx)
        legend.append(band_name)

    virt_wl, corr_matrix = read_corr_matrix(xml_vis)

    # # plot bands
    # fig, ax = plt.subplots(figsize=(20,10))
    # ax.set_aspect('auto')
    # ax.plot(wavelengths, responses)
    # ax.legend(legend)
    # plt.show()
    # # plt.savefig('nir.png', dpi=500)
