#!../venv/bin/python2

import glob
from IPython import embed
from exif import Image
from geopy.point import Point
import shapely.geometry as shg
import geopandas as gpd
import os
import argparse

# parse the arguments
parser = argparse.ArgumentParser()
parser.add_argument("--imgs_folder", "-i", required=True,
    help="Path to the folder where the geotagged images are stored.")
parser.add_argument("--output_folder", "-o", required=True,
    help="Path to the folder where the shapefile will be stored.")
parser.add_argument("--filename", "-n", default="locations.shp", required=False,
    help="Name to assign to the file.")
args = parser.parse_args()

imgs_path = args.imgs_folder
if imgs_path[-1] != '/':
    imgs_path += '/'

imgs_list = glob.glob(imgs_path + '*.jpg')

points = []
for img_name in imgs_list:
    with open(img_name, 'rb') as img:
        img = Image(img)
        lon = img.gps_longitude
        lat = img.gps_latitude
        location = "{} {}m {}s N {} {}m {}s E".format(lat[0], lat[1], lat[2],
            lon[0], lon[1], lon[2])
        point = Point(location)
        point = shg.Point(point.longitude, point.latitude)
        points.append(point)

# write the points to a shapefile
gsr = gpd.GeoSeries(points)
gsr.crs = {'init': 'epsg:4326'}
gsr = gsr.to_crs({'init': 'epsg:3857'})
output_folder = args.output_folder
filename = args.filename

if not filename.split('.')[-1] == 'shp':
    filename += '.shp'

if not os.path.exists(output_folder):
    os.mkdir(output_folder)

gsr.to_file(os.path.join(output_folder, filename))
