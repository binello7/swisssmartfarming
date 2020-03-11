#!venv3/bin/python3

import os
import argparse


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
args = parser.parse_args()

sep = os.path.sep
frames_folder = args.frames_folder

first_img = ''
last_img = ''
step = 2

frames_folder_levels = frames_folder.split(sep)
location = frames_folder_levels[-4]
date = frames_folder_levels[-3]
camera = frames_folder_levels[-1]

templates_folder = '/home/seba/Projects/swisssmartfarming/cfg/pix4d'
template_file = os.path.join(templates_folder, (camera + '.tmpl'))

date_folder = sep.join(frames_folder.split(sep)[:-2])

project_name = '_'.join((location, date, camera)) + '.p4d'
project_folder = os.path.join(date_folder, 'pix4d', camera)
project_file = os.path.join(project_folder, project_name)

if not os.path.isdir(project_folder):
    os.mkdir(project_folder)

pix4d_options = {
    '-c': '',
    '--stdout': '',
    '-n': '',
    '--image-dir=': frames_folder,
    '--template=': template_file
}

command = write_command(pix4d_options, project_file)

# run pix4d with new project -> project created but processing fails
os.system(command)

# update the pix4d_options
pix4d_options.pop('-n')
pix4d_options.update({'-r': ''})
command = write_command(pix4d_options, project_file)

os.system(command)
