#!/usr/bin/python2

from timeit import default_timer as timer
import rosbag
import yaml
import subprocess
import numpy as np
import argparse
import os

parser = argparse.ArgumentParser()
parser.add_argument('--bag_file', required=True,
                    help='Path to the bag-file where the rtk-data are contained')
parser.add_argument('--output_folder', required=True,
                    help='Path to the folder where the rtk-data will be stored')
args = parser.parse_args()

start = timer()
print('Saving rtk-data...')
topic = '/ssf/dji_sdk/rtk_position'

bag = rosbag.Bag(args.bag_file)
info_dict = yaml.load(subprocess.Popen(['rosbag', 'info', '--yaml', args.bag_file], stdout=subprocess.PIPE).communicate()[0])

rtk_info = filter(lambda rtk: rtk['topic'] == topic, info_dict['topics'])
n_msgs = rtk_info[0]['messages']

genBag = bag.read_messages(topic)

data = np.zeros([n_msgs, 4])
for i,b in enumerate(genBag):
    data[i,0] = b.message.header.stamp.to_nsec()
    data[i,1] = b.message.latitude
    data[i,2] = b.message.longitude
    data[i,3] = b.message.altitude

np.savetxt(os.path.join(args.output_folder, 'rtk_data.csv'), data,
            delimiter=",", header="timestamp[ns],latitude[deg],longitude[deg],altitude[m]")

end = timer()

print("Saving rtk-data took {:.1f}s\n".format(end-start))
