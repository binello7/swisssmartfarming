#!/usr/bin/python2

import subprocess, yaml
import rosbag
from cv_bridge import CvBridge
import cv2
import matplotlib.pyplot as plt
from PIL import Image
import os
import argparse

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Extract thermal images messages and saves them in .tiff format')
    parser.add_argument('--bag_file',
                            required=True,
                            help='Path to the folder containing the mosaic images to resample')
    parser.add_argument('--output_folder',
                            required=True,
                            help='Number of bands of the camera')
    args = parser.parse_args()

    topic = '/ssf/thermalgrabber_ros/image_deg_celsius'

    bag = rosbag.Bag(args.bag_file)
    info_dict = yaml.load(subprocess.Popen(['rosbag', 'info', '--yaml', args.bag_file], stdout=subprocess.PIPE).communicate()[0])
    print(info_dict)

    genBag = bag.read_messages(topic)

    for k,b in enumerate(genBag):
        print('saving file {} / {}'.format(k, info_dict['messages']))

        cb = CvBridge()
        cv_image = cb.imgmsg_to_cv2(b.message, b.message.encoding)

        data = cv_image[:,:]

        img = Image.fromarray(data)
        img.save(os.path.join(args.output_folder, 'frame_{:06}.tiff'.format(k)))
