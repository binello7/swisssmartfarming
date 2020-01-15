#!../venv/bin/python2
import os
from swisssmartfarming.tellnet.datainterface import DataInterface
from IPython import embed
import rasterio.plot as riop
import matplotlib.pyplot as plt

base_folder = "/media/seba/Samsung_2TB/AgriCircle/Zollikofen/20190618"

files_names = [
    "zollikofen_20190618_nir_transparent_reflectance_group1_georef.tif",
    "zollikofen_20190618_vis_transparent_reflectance_group1_georef.tif"
]

shp_file = "/media/seba/Samsung_2TB/AgriCircle/Zollikofen/Shapes/zollikofen.shp"

files_paths = []
for file_name in files_names:
    files_paths.append(os.path.join(base_folder, file_name))

zollikofen = DataInterface()

for file_path in files_paths:
    zollikofen.add_dataset(file_path)

zollikofen.add_shapefile(shp_file)

for dataset_name in zollikofen.datasets_names:
    zollikofen.crop_dataset(dataset_name)

zollikofen.align_datasets("20190618_vis")
#
# fig, axs = plt.subplots(1, 2)
#
# riop.show(zollikofen.datasets["20190618_nir"], with_bounds=True, ax=axs[0],
#     cmap="Greys_r")
# riop.show(zollikofen.datasets["20190618_vis"], with_bounds=True, ax=axs[1],
#     cmap="Greys_r")
# plt.show()
#

embed()
