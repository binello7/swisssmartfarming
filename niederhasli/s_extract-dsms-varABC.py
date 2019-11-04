#!../venv/bin/python2

"""Split Niederhasli-DSMs into subplots A_1, A_2, ... C_13, C_14

This script uses GDAL (gdalwarp) to split up the DSMs of the project
Niederhasli. The DSMs to split up should already have been reprojected
(for better matching) and aligned (to have matrices of same size).

The field 'Niederhasli' is divided in three plots (A, B, C) that are
again divided into 14 subplots. By executing the script the DSMs of
the subplots A_1, ..., A_14, B_1, ..., B_14 and C_1, ..., C_14 are
generated and placed into 'Niederhasli/dsms/dsms_{A/B/C}'

This script requires modules 'os' and 'ssf_functions'
"""

import os
import functions.ssf_functions as ssf

# input DSMs: all field, reprojected, aligned, 3 dates
dsm_date1 = ("/media/seba/Samsung_2TB/Analysis/QGIS/Niederhasli/dsms/dsms_crop/"
            "niederhasli_20190527_rgb_dsm_crop.tif")
dsm_date2 = ("/media/seba/Samsung_2TB/Analysis/QGIS/Niederhasli/dsms/dsms_crop/"
            "niederhasli_20190719_rgb_dsm_crop.tif")
dsm_date3 = ("/media/seba/Samsung_2TB/Analysis/QGIS/Niederhasli/dsms/dsms_crop/"
            "niederhasli_20191007_rgb_dsm_crop.tif")
dsm_dates = [dsm_date1, dsm_date2, dsm_date3]

# input shapes as cutline
shp_path = "/media/seba/Samsung_2TB/Analysis/QGIS/Niederhasli/shapes"
shps_folders = ["var_A", "var_B", "var_C"]

# output folders
dsms_pathout = "/media/seba/Samsung_2TB/Analysis/QGIS/Niederhasli/dsms/"
dsms_out_folders = ["dsms_A", "dsms_B", "dsms_C"]

px_res = 0.00814116 # m/px

for n, var in enumerate(shps_folders):
    shps_folder = os.path.join(shp_path, var)
    shps_list = os.listdir(shps_folder)
    dsms_out_folder = os.path.join(dsms_pathout, dsms_out_folders[n])
    if not os.path.isdir(dsms_out_folder):
        os.mkdir(dsms_out_folder)

    # remove files != *.shp
    for file in shps_list[:]:
        if file.split(".")[-1] != "shp":
            shps_list.remove(file)

    for dsm in dsm_dates:
        for n_shp, shp in enumerate(shps_list):
            shp_full = os.path.join(shps_folder, shp)
            dsm_base = dsm.split("/")[-1]
            dsm_base = dsm_base.split("_")[0:-1]
            dsm_base = (("_".join(dsm_base) + "_"
                        + shp.split(".")[0].split("_")[0] + '{:02d}' + ".tif")
                        .format(n_shp+1))
            dsm_out = os.path.join(dsms_out_folder, dsm_base)
            ssf.gdal_crop(dsm, shp_full, px_res, px_res, dsm_out)
