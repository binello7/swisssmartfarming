#!../venv/bin/python2

from __future__ import division
import numpy as np
import utils.ssf_functions as ssf
import matplotlib.pyplot as plt
from osgeo import gdal
import os

dsms_folderA = "/media/seba/Samsung_2TB/Analysis/QGIS/Niederhasli/dsms/dsms_A"
dsms_folderB = "/media/seba/Samsung_2TB/Analysis/QGIS/Niederhasli/dsms/dsms_B"
dsms_folderC = "/media/seba/Samsung_2TB/Analysis/QGIS/Niederhasli/dsms/dsms_C"
dsms_folders = [dsms_folderA, dsms_folderB, dsms_folderC]
variants = ["A", "B", "C"]

# prepare the figure where the different dsms will be plotted
n_vars = len(variants)
n_plots = 14
n_tot = n_vars * n_plots

my_dpi = 96

fig, axs = plt.subplots(nrows=n_plots, ncols=n_vars, figsize=(30, 100))

count = 0
for n_folder, folder in enumerate(dsms_folders):
    for i in range(n_plots):
        plot_n = "{:02d}".format(i+1)
        name_initial = ("niederhasli_20190527_rgb_dsm_{}{}.tif"
            .format(variants[n_folder], plot_n))
        path_initial = os.path.join(folder, name_initial)
        dsm_initial = ssf.read_geotiff(path_initial)
        raster = gdal.Open(path_initial)
        X_res = raster.GetGeoTransform()[1]
        Y_res = -raster.GetGeoTransform()[5]
        mask_plot = dsm_initial!=0
        dsm_initial = dsm_initial - np.min(dsm_initial[mask_plot])

        name_final = ("niederhasli_20191007_rgb_dsm_{}{}.tif"
            .format(variants[n_folder], plot_n))
        dsm_final = ssf.read_geotiff(os.path.join(folder, name_final))
        dsm_final = dsm_final - np.min(dsm_final[mask_plot])

        dsm_corr = dsm_final - dsm_initial
        soy_thr = 0.5#m
        dsm_corr[dsm_corr<soy_thr] = 0
        Vol = np.sum(dsm_corr) * X_res * Y_res
        dsm_corr[~mask_plot] = np.nan

        g = axs[i, n_folder].imshow(dsm_corr)
        axs[i, n_folder].set_title(u'plot_{}{} | V = {:.2f} m\u00B3'
            .format(variants[n_folder], plot_n, Vol))
        axs[i, n_folder].set_yticks([])
        axs[i, n_folder].set_xticks([])
        cbar = fig.colorbar(g, ax=axs[i, n_folder])
        # ticklabs = cbar.ax.get_yticklabels()
        # cbar.ax.set_yticklabels(ticklabs, fontsize=5)
        count+=1
        print("Working... {:.2f}%".format(count/n_tot * 100))


plt.tight_layout()
plt.subplots_adjust(top=0.95, bottom=0.05)
plt.savefig('volumes_05.png', dpi=2*my_dpi)
