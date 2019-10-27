import os

# crop DSMs for variants 1-14
## input_paths
dsm_date1 = "/media/seba/Samsung_2TB/Analysis/QGIS/Niederhasli/dsm/niederhasli_20190527_rgb_dsm_crop.tif"
dsm_date2 = "/media/seba/Samsung_2TB/Analysis/QGIS/Niederhasli/dsm/niederhasli_20190719_rgb_dsm_crop.tif"
dsm_date3 = "/media/seba/Samsung_2TB/Analysis/QGIS/Niederhasli/dsm/niederhasli_20191007_rgb_dsm_crop.tif"
dsms_pathout = "/media/seba/Samsung_2TB/Analysis/QGIS/Niederhasli/dsm/dsms_C"

px_res = 0.00814116 # m/px

if not os.path.isdir(dsms_pathout):
    os.mkdir(dsms_pathout)

dsm_dates = [dsm_date1, dsm_date2, dsm_date3]

Cvar_path = "/media/seba/Samsung_2TB/Analysis/QGIS/Niederhasli/shapes/var_C"
Cvar_shapes_list = os.listdir(Cvar_path)

for Cvar in Cvar_shapes_list[:]:
    print(Cvar, count)
    count+=1
    if Cvar.split(".")[-1] != "shp":
        Cvar_shapes_list.remove(Cvar)

Cvar_shapes_list.sort()


for dsm in dsm_dates:
    for Cvar in Cvar_shapes_list:
        shp = os.path.join(Cvar_path, Cvar)
        dsm_base = dsm.split("/")[-1]
        dsm_base = dsm_base.split("_")[0:-1]
        dsm_base = "_".join(dsm_base) + "_" + Cvar.split(".")[0] + ".tif"
        sys_cmd = ("gdalwarp -of GTiff -tr {} {} -tap -cutline"
                   " {} -cl {} -crop_to_cutline -dstnodata 0.0 {} {}").format(
            px_res,
            px_res,
            shp,
            Cvar.split(".")[0],
            dsm,
            os.path.join(dsms_pathout, dsm_base)
        )
        os.system(sys_cmd)
