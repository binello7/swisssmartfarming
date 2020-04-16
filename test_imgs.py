#!venv3/bin/python3

import utils.functions as ufunc
from IPython import embed

img_path = "/media/seba/Samsung_2TB/Forest-Project/datasets/test_20200411/frames/photonfocus_nir/frame_00111.tif"

img_array = ufunc.read_geotiff(img_path)
embed()
