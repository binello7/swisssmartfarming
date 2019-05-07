#!/usr/bin/env python

# ------------------------------------------------------------------------------
# Function : restamp ros bagfile (using trigger timestamps)
# Method   : uses the closest prior trigger timestamp to restamp the image message header
# Project  : SSF
# Author   : www.asl.ethz.ch
# Version  : V01  10APR2019 Initial version.
# Comment  :
# Status   : under review
#
# Usage    : python restamp_images_with_triggers.py -i inbag.bag -o outbag.bag
# ------------------------------------------------------------------------------

import roslib
import rosbag
import rospy
import numpy as np
import sys
import getopt
from std_msgs.msg import String
from sensor_msgs.msg import Image
from arduino_vi_sync.msg import TimeNumbered
# from IPython import embed

def get_parent_id(parent_array, mask, subset_id):
    return np.arange(parent_array.shape[0])[mask][subset_id]

def find_nearest(array, value):
    parent_array = value - array
    mask = (parent_array > 0)
    if parent_array[mask].size > 0:
        subset_id = np.argmin(parent_array[mask])
        print "theader- trigger: ", parent_array[mask][subset_id]
    else:
        return 0
    return array[get_parent_id(parent_array, mask, subset_id)]

def main(argv):

    inputfile = ''
    outputfile = ''
    # image_topic = '/mono/image_raw'
    # image_topic = '/ssf/BFS_usb_0/image_raw'
    # image_topic = '/ssf/photonfocus_camera_vis_node/image_raw'
    image_topic = '/ximea_asl/image_raw'
    # trigger_topic = '/arduino/trigger'
    trigger_topic = '/arduino_vi/img_time'

    # parse arguments
    try:
        opts, args = getopt.getopt(argv,"hi:o:",["ifile=","ofile="])
    except getopt.GetoptError:
        print 'usage: restamp_images_with_triggers.py -i <inputfile> -o <outputfile>'
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print 'usage: python restamp_images_with_triggers.py -i <inputfile> -o <outputfile>'
            sys.exit()
        elif opt in ("-i", "--ifile"):
            inputfile = arg
        elif opt in ("-o", "--ofile"):
            outputfile = arg

    # print console header
    print ""
    print "restamp_bag"
    print ""
    print 'input file:  ', inputfile
    print 'output file: ', outputfile
    print ""
    print "starting restamping (may take a while)"
    print ""

    outbag = rosbag.Bag(outputfile, 'w')
    tprev = 0
    try:
        trigger_timestamps = np.array([])
        for topic, msg, t in rosbag.Bag(inputfile).read_messages(topics=[trigger_topic]):
            trigger_timestamps=np.append(trigger_timestamps, msg.time.to_sec())
            # Write message in output bag with input message header stamp
            print "Trigger: trecord - theader: ", t.to_sec() - msg.time.to_sec()
        #embed()
        for topic, msg, t in rosbag.Bag(inputfile).read_messages(topics=[image_topic]):
            image_trigger_ts = find_nearest(trigger_timestamps, msg.header.stamp.to_sec())
            print "Image: trecord - theader: ", t.to_sec() - msg.header.stamp.to_sec()
            if image_trigger_ts > 0:
                msg.header.stamp = rospy.Time.from_sec(image_trigger_ts)
                outbag.write(topic, msg, msg.header.stamp)
        for topic, msg, t in rosbag.Bag(inputfile).read_messages(topics=[trigger_topic]):
            outbag.write(topic, msg, msg.time)


    # print console footer
    finally:
        print ""
        print ""
        print "finished iterating through input bag"
        print "output bag written"
        print ""
        outbag.close()

if __name__ == "__main__":
    main(sys.argv[1:])
