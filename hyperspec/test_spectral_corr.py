#!../venv/bin/python2

import hyperspec as hyp
from IPython import embed
import matplotlib.pyplot as plt

filepath = "/home/seba/polybox/Matterhorn.Project/Pictures/Photonfocus.samples/470-620nm_filter/SSM4x4_colors_mono10_11ms.raw"
filepath2 = "/home/seba/polybox/Matterhorn.Project/Pictures/Photonfocus.samples/470-620nm_filter/no_ref/uncorrected/SSM4x4_colors_mono10_11ms_uncorrected_hc.hdr"

xml_path = "/home/seba/polybox/Matterhorn.Project/Pictures/Photonfocus.samples/CMV2K-SSM4x4-470_620-9.2.18.8.xml"

img = hyp.read_bsq(filepath2)
raw = hyp.read_raw(filepath)

wavelengths, corr_matrix = hyp.read_corr_matrix(xml_path)

embed()
