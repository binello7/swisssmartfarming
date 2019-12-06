#!./venv/bin/python2

import utils.ssf_functions as ssf
import numpy as np
import os
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--date_folder', required=True,
                    help='Path to the folder containing the different cameras subfolders')
args = parser.parse_args()


s_nir = "nir"
s_vis = "vis"
nir_folder = os.path.join(args.date_folder, s_nir)
vis_folder = os.path.join(args.date_folder, s_vis)


# get average DN values of 18% reflectance patch in colorchecker classic
DNpanel_nir = ssf.get_DNpanel(camera_type=s_nir, initialdir=nir_folder)
DNpanel_vis = ssf.get_DNpanel(camera_type=s_vis, initialdir=vis_folder)

# save the average DN values to a file
np.savetxt(os.path.join(args.date_folder, 'DNpanel18_nir.csv'), DNpanel_nir,
            delimiter=",", header="Average DN values 18% reflectance gray patch for nir-camera")

np.savetxt(os.path.join(args.date_folder, 'DNpanel18_vis.csv'), DNpanel_vis,
            delimiter=",", header="Average DN values 18% reflectance gray patch for vis-camera")

#-------------------------------------------------------------------------------
# # read average DN values 18% reflectance patch from files
# DNpanel_nir = np.genfromtxt(os.path.join(project_folder, 'DNpanel18_nir.csv'),
#                                 delimiter=',', skip_header=1)
#
# DNpanel_vis = np.genfromtxt(os.path.join(project_folder, 'DNpanel18_vis.csv'),
#                                 delimiter=',', skip_header=1)
#-------------------------------------------------------------------------------

# convert DN values of the images to reflectance values
input_folders = [nir_folder, vis_folder]
DNpanels = [DNpanel_nir, DNpanel_vis]
bands = [25, 16]
ref_midgray = 0.18
for f in range(2):
    img_list = sorted(os.listdir(input_folders[f]))

    output_folder = os.path.join(input_folders[f], "ref")
    if not os.path.isdir(output_folder):
        os.mkdir(output_folder)

    # remove files that are not images
    for img in img_list[:]: #img_list[:] makes a copy of img_list
        if not (img.startswith('frame_')):
            img_list.remove(img)

    # loop through every image in the folder and convert it to reflectnace
    for img in img_list:
        print("convert to reflectance " + img)
        img_array = ssf.read_geotiff(os.path.join(input_folders[f], img))
        ref_array = np.zeros(img_array.shape)

        for b in range(bands[f]):
            ref_array[:,:,b] = img_array[:,:,b]/DNpanels[f][b] * ref_midgray * 100
            ssf.write_geotiff(ref_array, os.path.join(output_folder, img))
