#!./venv/bin/python2

from timeit import default_timer as timer
import rosbag
import yaml
import subprocess
import numpy as np
import argparse
import os

parser = argparse.ArgumentParser()
parser.add_argument('--topic', required=True,
                    help='Image topic for which the image-timestamps will be extracted')
parser.add_argument('--bag_file', required=True,
                    help='Path to the bag-file containing the image topic')
parser.add_argument('--output_folder', required=True,
                    help='Path to the folder where the timestamps file will be stored')
args = parser.parse_args()

start = timer()
print('Saving {} timestamps...'.format(args.topic))

bag = rosbag.Bag(args.bag_file)
info_dict = yaml.load(subprocess.Popen(['rosbag', 'info', '--yaml', args.bag_file], stdout=subprocess.PIPE).communicate()[0])

tstamps_info = filter(lambda tstamps: tstamps['topic'] == args.topic, info_dict['topics'])
n_msgs = tstamps_info[0]['messages']

genBag = bag.read_messages(args.topic)

data = np.zeros([n_msgs, 1])
for i,b in enumerate(genBag):
    data[i,0] = b.message.header.stamp.to_nsec()

np.savetxt(os.path.join(args.output_folder, 'img_tstamps.csv'), data,
            delimiter=",", header="timestamp[ns]")

end = timer()

print("Saving images timestamps took {:.1f}s\n".format(end-start))
