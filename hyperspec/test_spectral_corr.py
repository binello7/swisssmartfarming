#!../venv/bin/python2

import hyperspec as hyp
from IPython import embed
import matplotlib.pyplot as plt
from swisssmartfarming.resample_mosaics import resample
import numpy as np

filepath = "/home/seba/polybox/Matterhorn.Project/Pictures/Photonfocus.samples/470-620nm_filter/SSM4x4_colors_mono10_11ms.raw"
no_ref = "/home/seba/polybox/Matterhorn.Project/Pictures/Photonfocus.samples/470-620nm_filter/no_ref/uncorrected/SSM4x4_colors_mono10_11ms_uncorrected_hc.raw"

xml_path = "/home/seba/polybox/Matterhorn.Project/Pictures/Photonfocus.samples/CMV2K-SSM4x4-470_620-9.2.18.8.xml"

raw = hyp.read_raw(filepath)

wavelengths, corr_matrix = hyp.read_corr_matrix(xml_path)
nb_bands = 16

# no ref
img_noref = hyp.read_bsq(no_ref)
print(img_noref.shape)
img_res = resample(raw, nb_bands)
print(img_res.shape)
embed()

# plots
fig, ax = plt.subplots(1,2)
ax[0].imshow(np.squeeze(img_noref[:,:,0]))
ax[1].imshow(np.squeeze(img_res[:,:,0]))
plt.show()
