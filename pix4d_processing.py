#!venv2/bin/python2

import os
import argparse
import shutil
import rootpath
import utils.functions as ufunc
from glob import glob


def write_command(pix4d_options, project_file, exe='pix4dmapper'):
    exe += ' '
    for opt in pix4d_options.items():
        exe += ''.join(opt) + ' '
    command = exe + project_file
    return command
#-------------------------------------------------------------------------------

# construct ArgumentParser
parser = argparse.ArgumentParser()
parser.add_argument('--frames_folder', '-f',
    help='Path to the folder containing the images to process',
    required=True)
parser.add_argument('--idx_first_img',
    help=("Index of first image to process. The first image in the folder has "
        "index '0', the last one has index '-1'"),
    type=int,
    required=False,
    default=0)
parser.add_argument('--idx_last_img',
    help=("Index of last image to process. The first image in the folder has "
        "index '0', the last one has index '-1'"),
    type=int,
    required=False,
    default=-1)
parser.add_argument('--imgs_step',
    help=("Step between consecutive images. If 'imgs_step=1' every image "
        "between 'idx_first_img' and 'idx_last_img' will be chosen"),
    type=int,
    required=False,
    default=1)
args = parser.parse_args()

# set needed variables
sep = os.path.sep
frames_folder = args.frames_folder
frames_folder = ufunc.rm_sep(frames_folder)
idx_first_img = args.idx_first_img
idx_last_img = args.idx_last_img
imgs_step = args.imgs_step

# select the desired images
imgs_list = glob(os.path.join(frames_folder, '*.jpg'))
imgs_list.extend(glob(os.path.join(frames_folder, '*.tif')))
imgs_list.sort()
if idx_last_img == -1:
    idx_last_img = None
else:
    idx_last_img += 1
imgs_list = imgs_list[idx_first_img:idx_last_img:imgs_step]

# copy the selected images to a tmp folder
tmp_folder = os.path.join(frames_folder, 'tmp')
if not os.path.isdir(tmp_folder):
    os.makedirs(tmp_folder)
print('Copying images to ./tmp folder...')
for img_src in imgs_list:
    img_dst = os.path.join(tmp_folder, img_src.split(sep)[-1])
    shutil.copy(img_src, img_dst)

# get the root_folder
root_folder = rootpath.detect()

# get location, date, camera
frames_folder_levels = frames_folder.split(sep)
location = frames_folder_levels[-4]
date = frames_folder_levels[-3]
camera = frames_folder_levels[-1]

# get the pix4d template
templates_folder = os.path.join(root_folder, 'cfg', 'pix4d')
template_file = os.path.join(templates_folder, (camera + '.tmpl'))

# define the project file and create the project folder
date_folder = sep.join(frames_folder.split(sep)[:-2])
project_name = '_'.join((location, date, camera)) + '.p4d'
project_folder = os.path.join(date_folder, 'pix4d', camera)
project_file = os.path.join(project_folder, project_name)
if not os.path.isdir(project_folder):
    os.makedirs(project_folder)

# generate command to create the new pix4d project
pix4d_options = {
    '-c': '',
    '--stdout': '',
    '-n': '',
    '--image-dir=': tmp_folder,
    '--template=': template_file
}
command = write_command(pix4d_options, project_file)

# run the command - project_file is created but processing fails
print("executing command '{}'".format(command))
os.system(command)

# update the pix4d_options and generate the new command to process the project
pix4d_options.pop('-n')
pix4d_options.update({'-r': ''})
command = write_command(pix4d_options, project_file)

# run the command with the previously created project - processing should run
print("executing command '{}'".format(command))
os.system(command)

# remove tmp folder
print('Removing ./tmp folder...')
shutil.rmtree(tmp_folder)
