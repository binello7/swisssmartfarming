#!../venv2/bin/python2

import hyperspec as hyp
from IPython import embed
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns

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

# read all images
## read raw
raw = hyp.read_raw(raw_path)
white = hyp.read_raw(white_path)
dark_tw = hyp.read_raw(dark_tw_path)
dark_to = hyp.read_raw(dark_to_path)

## read bsq
uncorr = hyp.read_bsq(uncorrected)
ref_pf = hyp.read_bsq(reflectance)
ref_pf = ref_pf / 2**15
corr_ref = hyp.read_bsq(corr_reflectance)

# compute reflectance from raw
hs_img = hyp.HypSpecImage("photonfocus_vis", raw)
hs_img.rad_to_refl(white, dark_to, dark_tw, ti, tw)
hs_img.reshape()
hs_img.median_filter()
# embed()

# assert (img_res==img_pf).all()

# remove NaNs
ref_med_nonan = hs_img.med[~np.isnan(hs_img.med)]
ref_nonan = hs_img.res[~np.isnan(hs_img.res)]

# plots
## images
fig, ax = plt.subplots(1,3, figsize=(15, 5))
fontsize = 10
ax[0].imshow(np.squeeze(ref_pf[:,:,0]))
ax[0].set_title('Photonfocus Wizard, uncorrected', fontsize=fontsize)
ax[1].imshow(np.squeeze(hs_img.med[:,:,0]))
ax[1].set_title('Own implementation, 3x3 median filter, uncorrected', fontsize=fontsize)
ax[2].imshow(np.squeeze(hs_img.res[:,:,0]))
ax[2].set_title('Own implementation, no median filter, uncorrected', fontsize=fontsize)
plt.savefig('img_band1_comparison.png', dpi=400)

# histograms
fig, ax = plt.subplots(1,3, figsize=(15, 5))
sns.distplot(ref_pf.flatten(), ax=ax[0])
sns.distplot(ref_med_nonan.flatten(), ax=ax[1])
sns.distplot(ref_nonan.flatten(), ax=ax[2])
ax[0].set_title('Photonfocus Wizard, uncorrected', fontsize=fontsize)
ax[1].set_title('Own implementation, 3x3 median filter, uncorrected', fontsize=fontsize)
ax[2].set_title('Own implementation, no median filter, uncorrected', fontsize=fontsize)
# plt.show()
plt.savefig('hist_comparison.png', dpi=400)
