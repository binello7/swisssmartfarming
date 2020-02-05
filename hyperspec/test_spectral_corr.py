#!../venv/bin/python2

from __future__ import division
import hyperspec as hyp
from IPython import embed
import matplotlib.pyplot as plt
import numpy as np

raw_path = "/home/seba/polybox/Matterhorn.Project/Pictures/Photonfocus.samples/470-620nm_filter/SSM4x4_colors_mono10_11ms.raw"
dark_tw_path = "/home/seba/polybox/Matterhorn.Project/Pictures/Photonfocus.samples/470-620nm_filter/SSM4x4_dark_ref_mono10_9ms.raw"
dark_to_path = "/home/seba/polybox/Matterhorn.Project/Pictures/Photonfocus.samples/470-620nm_filter/SSM4x4_dark_ref_mono10_11ms.raw"
white_path = "/home/seba/polybox/Matterhorn.Project/Pictures/Photonfocus.samples/470-620nm_filter/SSM4x4_white_ref_mono10_9ms.raw"
xml_path = "/home/seba/polybox/Matterhorn.Project/Pictures/Photonfocus.samples/CMV2K-SSM4x4-470_620-9.2.18.8.xml"
uncorrected = "/home/seba/polybox/Matterhorn.Project/Pictures/Photonfocus.samples/470-620nm_filter/with_white_dark_ref/uncorrected/SSM4x4_colors_mono10_11ms_uncorrected_hc.raw"

reflectance = "/home/seba/polybox/Matterhorn.Project/Pictures/Photonfocus.samples/470-620nm_filter/with_white_dark_ref/reflectance/SSM4x4_colors_mono10_11ms_reflectance_hc.raw"

corr_reflectance = "/home/seba/polybox/Matterhorn.Project/Pictures/Photonfocus.samples/470-620nm_filter/with_white_dark_ref/corrected_reflectance/SSM4x4_colors_mono10_11ms_corrected_reflectance_hc.raw"

ti = 11
tw = 9

raw = hyp.read_raw(raw_path)
white = hyp.read_raw(white_path)
dark_tw = hyp.read_raw(dark_tw_path)
dark_to = hyp.read_raw(dark_to_path)

uncorr = hyp.read_bsq(uncorrected)
ref_pf = hyp.read_bsq(reflectance)
ref_pf = ref_pf / 2**16
corr_ref = hyp.read_bsq(corr_reflectance)

ref = hyp.rad_to_refl(raw, white, dark_to, dark_tw, ti, tw)

hs_img = hyp.HypSpecImage("photonfocus_vis")

raw_res = hs_img.reshape(raw)
ref_res = hs_img.reshape(ref)
# assert (img_res==img_pf).all()
embed()

# # plots
# fig, ax = plt.subplots(1,2)
# ax[0].imshow(np.squeeze(img_noref[:,:,0]))
# ax[1].imshow(np.squeeze(img_res[:,:,0]))
# plt.show()
