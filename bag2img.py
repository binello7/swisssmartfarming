#!./venv/bin/python3

import os
import argparse
# from ros import rosbag
# import rospy
# from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
from timeit import default_timer as timer
from IPython import embed

# def bag2imgs()

def run(self):
    start = timer()
    if not os.path.exists(args.output_folder):
        os.mkdir(args.output_folder)
    bag_file=rosbag.Bag(args.bag_file,"r")

    for i,msg in enumerate(bag_file.read_messages(topics=[args.topic])):
        # try:
        cv2_img = bridge.imgmsg_to_cv2(msg.message, desired_encoding=args.encoding)
        outputFileName=os.path.join(args.output_folder,"{}_{:06d}.{}".format(args.file_name,i,args.output_format))
        print("{} saved".format(outputFileName))
        total_n_image=i
        cv2.imwrite(outputFileName,cv2_img)
        # except CvBridgeError, e:
        #     print(e)
    bag_file.close()
    end = timer()
    print("=====================================================")
    print("Extraction took {:.03f}s for extracting {} images".format(end - start,total_n_image+1))
    print("=====================================================")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Extract images from a bag file')
    parser.add_argument('--topic', '-t',
        required=True,
        help='Name of image topic to extract')
    parser.add_argument('--bag_file', '-b',
        required=True,
        help='Path to the bag file and name, e.g. ./dataset/Big.bag')
    parser.add_argument('--file_name',
        required=False,
        help='Prefixed file name for stored images',
        default='frame')
    parser.add_argument('--output_format',
        required=False,
        help='output image format, e.g., jpg or png',
        default='png',
        choices=['png', 'jpg'])
    parser.add_argument('--output_folder', '-o',
        required=False,
        help='Path to a folder where the extracted images will be stored.',
        default='./output')
    parser.add_argument('--encoding',
        required=False,
        help='encoding options, e.g., mono8, mono16, bgr8, rgb8, bgra8, rgba8',
        default="passthrough")
    args = parser.parse_args()
    embed()

    # extractor=Img_Extractor()
    # extractor.run()
