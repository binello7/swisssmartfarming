#!../venv/bin/python2

"""Compute total subplot volume for every one of the 14 subplots of A, B, C.

This script reads the DSMs of the 14 subplots of variants A, B, C , reduces the
absolute DSM to a relative DSM, and subtracts the DSM at the earliest date in
order to obtain plants total volumes without the influence of the terrain. Those
volumes, along with the max plant height per subplot, are then written to an
xlsx-file.

This script requires modules 'numpy', 'ssf_functions', 'gdal', 'os' and 'pandas'
"""

import numpy as np
import functions.ssf_functions as ssf
from osgeo import gdal
import os
import pandas as pd

dsms_parent = "/media/seba/Samsung_2TB/Analysis/QGIS/Niederhasli/dsms"
dsms_folderA = "/media/seba/Samsung_2TB/Analysis/QGIS/Niederhasli/dsms/dsms_A"
dsms_folderB = "/media/seba/Samsung_2TB/Analysis/QGIS/Niederhasli/dsms/dsms_B"
dsms_folderC = "/media/seba/Samsung_2TB/Analysis/QGIS/Niederhasli/dsms/dsms_C"
dsms_folders = [dsms_folderA, dsms_folderB, dsms_folderC]

for n_folder, dsms_folder in enumerate(dsms_folders):
    dsms_files = os.listdir(dsms_folder)

    dates = []
    plots = []
    for dsm in dsms_files:
        date = dsm.split("_")[1]
        plot = dsm.split(".")[0].split("_")[-1]
        plots.append(plot)
        dates.append(date)
    plots = list(set(plots))
    dates = list(set(dates))
    dates = sorted(dates)
    plots = sorted(plots)

    # declare variables for final results
    if not 'volumes' in globals():
        volumes = np.zeros((len(plots), len(dates)-1, len(dsms_folders)))
        h_max = np.zeros(volumes.shape)

    for n_plot, plot in enumerate(plots):
        for n_date, date in enumerate(dates):
            dsms_file = "niederhasli_{}_rgb_dsm_{}.tif".format(date, plot)
            filepath = os.path.join(dsms_folder, dsms_file)
            dsm = ssf.read_geotiff(filepath)
            raster = gdal.Open(filepath)
            X_res = raster.GetGeoTransform()[1]
            Y_res = -raster.GetGeoTransform()[5]
            n_dates = len(dates)
            if ((n_date+n_dates%n_dates==0)):
                mask_plot = dsm!=0
                dsms = np.zeros(dsm.shape + (n_dates,))

            dsms[:,:,n_date] = dsm

            # verify if last available date for given plot
            if ((n_date+1)%n_dates==0):
                for i in range(n_dates):
                    min_dsm = np.min(dsms[:,:,i][mask_plot])
                    dsms[:,:,i][mask_plot] = dsms[:,:,i][mask_plot]-min_dsm

                dsm_ref = dsms[:,:,0].copy()

                for i in range(n_dates-1):
                    dsms[:,:,i+1] = dsms[:,:,i+1] - dsm_ref
                    # by subtracting dsm_ref some values get < 0. Set them to 0
                    dsms[:,:,i+1][dsms[:,:,i+1]<0] = 0
                    # undersown crop shouldn't be included in volume. This is
                    # less hight than hemp. Set everything < threshold to 0
                    h_threshold = 0.3 #m
                    dsms[:,:,i+1][dsms[:,:,i+1]<h_threshold] = 0
                    max_dsm = np.max(dsms[:,:,i+1][mask_plot])
                    h_max[n_plot, i, n_folder] = max_dsm
                    volumes[n_plot, i, n_folder] = np.sum(dsms[:,:,i+1]) * X_res * Y_res

vol_str = 'volume'
h_str = 'max height'

idx_vol = np.array([vol_str, vol_str, vol_str,
                    vol_str, vol_str, vol_str])

idx_h = np.array([h_str, h_str, h_str, h_str, h_str, h_str])
idx_var = np.array(['varA', 'varA', 'varB', 'varB', 'varC', 'varC'])
idx_date = np.array(['2019/07/19', '2019/10/07', '2019/07/19', '2019/10/07', '2019/07/19', '2019/10/07'])

indexes = [np.concatenate((idx_vol, idx_h)),
           np.concatenate((idx_var, idx_var)),
           np.concatenate((idx_date, idx_date))]

volumes_cp = volumes.copy()
h_max_cp = h_max.copy()
volumes = np.zeros(((len(dates)-1)*len(dsms_folders),len(plots)))
h_max = np.zeros(volumes.shape)
for i in range(len(dsms_folders)):
    volumes[(i*2):(i*2+2), :] = np.transpose(volumes_cp[:,:,i])
    h_max[(i*2):(i*2+2), :] = np.transpose(h_max_cp[:,:,i])

# correct volume C08 - only one row is considered
volumes[4:6, 7] = 2 * volumes[4:6, 7]

data = pd.DataFrame(np.concatenate((volumes, h_max)), index=indexes)
data.columns = np.arange(1,15)
# save dataframe
data.to_excel(os.path.join(dsms_parent, "volumes.xlsx"))
