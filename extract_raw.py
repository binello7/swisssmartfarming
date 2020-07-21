#!/usr/bin/env python3

# from processing.preprocessing import BasePreprocessor, SpectralProcessor
import utils.functions as ufunc
import os
import numpy as np
import matplotlib.pyplot as plt

from IPython import embed

# file_path = "/media/seba/Verbatim_1TB/matterhorn-project/reckenholz/20200518/flight1/bag/2020-05-18-12-01-21.bag"
# out_folder = "/home/seba/Downloads/raw_vis"
xml_vis = ("/home/seba/Projects/swisssmartfarming/cfg/cameras/photonfocus_vis/"
    "CMV2K-SSM4x4-470_620-9.4.10.10_HS03-VIS.xml")
xml_nir = ("/home/seba/Projects/swisssmartfarming/cfg/cameras/photonfocus_nir/"
    "CMV2K-SSM5x5-665_975-13.8.15.1_HS02-NIR.xml")
#
# basepreprocessor = BasePreprocessor(file_path)
# cam = 'photonfocus_vis'
#
# msgs = basepreprocessor.read_img_msgs(basepreprocessor.imgs_topics[cam])
# for i, msg in enumerate(msgs):
#     img_array = basepreprocessor.imgmsg_to_cv2(msg)
#     extension = '.tif'
#     filename = 'frame' + '_{:05d}'.format(i) + extension
#     filepath = os.path.join(out_folder, filename)
#     ufunc.write_geotiff(img_array, filepath)

vis = {
    'xml_file': xml_nir,
    'name': 'nir_resp.png',
    'range': (400, 1000)
}

nir = {
    'xml_file': xml_vis,
    'name': 'vis_resp.png',
    'range': (400, 1000)
}

def run(params):
    bands, opt_component = ufunc.read_responses(params['xml_file'])
    bands = np.transpose(bands) * 100
    wavelengths = range(params['range'][0], params['range'][1]+1)

    leg = ['band_{}'.format(i+1) for i in range(bands.shape[1])]

    fig = plt.figure(figsize=(13,8))
    plt.plot(wavelengths, bands)
    plt.legend(leg, loc='upper right')
    plt.xlabel('wavelength [nm]')
    plt.ylabel('QE [%]')
    plt.savefig(params['name'], bbox_inches='tight')

run(vis)
