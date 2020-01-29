#!./venv/bin/python2

from __future__ import print_function
import os
import argparse
from ros import rosbag
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
from timeit import default_timer as timer

class Img_Extractor(object):
    def __init__(self):
        self.parser = argparse.ArgumentParser(description='extract images from a bag file')
        self.args=None
        self.args_parse()
        self.bag_file=None
        self.bridge=CvBridge()
        self.total_n_image=None

    def args_parse(self):
        self.parser.add_argument('--topic', required=True,
            help='Name of image topic you want to extract')
        self.parser.add_argument('--bag_file', required=True,
            help='Path to the bag file and name, e.g. ./dataset/Big.bag')
        self.parser.add_argument('--file_name', required=False,
            help='Prefixed file name for stored images',
            default="frame")
        self.parser.add_argument('--output_format', required=False,
            choices=['png', 'jpg']
            help='output image format, either jpg or png',
            default="png")
        self.parser.add_argument('--output_folder', required=False,
            help='Path to a output folder where extracted images will be stored.',
            default="./output")
        self.parser.add_argument('--encoding', required=False,
            help='encoding options, e.g., mono8, mono16, bgr8, rgb8, bgra8, rgba8',
            default="passthrough")
        self.args = self.parser.parse_args()

    def run(self):
        start = timer()
        if not os.path.exists(self.args.output_folder):
            os.mkdir(self.args.output_folder)
        self.bag_file=rosbag.Bag(self.args.bag_file,"r")

        for i,msg in enumerate(self.bag_file.read_messages(topics=[self.args.topic])):
            try:
                cv2_img = self.bridge.imgmsg_to_cv2(msg.message, desired_encoding=self.args.encoding)
                outputFileName=os.path.join(self.args.output_folder,"{}_{:06d}.{}".format(self.args.file_name,i,self.args.output_format))
                print("{} saved".format(outputFileName))
                self.total_n_image=i
                cv2.imwrite(outputFileName,cv2_img)
            except CvBridgeError, e:
                print(e)
        self.bag_file.close()
        end = timer()
        print("=====================================================")
        print("Extraction took {:.03f}s for extracting {} images".format(end - start,self.total_n_image+1))
        print("=====================================================")

if __name__ == "__main__":
    extractor=Img_Extractor()
    extractor.run()
