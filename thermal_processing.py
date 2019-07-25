import subprocess, yaml
import rosbag
from cv_bridge import CvBridge
import cv2
import matplotlib.pyplot as plt

bag_file = "/media/seba/Samsung_2TB/Datasets/gorner_glacier/190709/2019-07-09-21-19-08.bag"
read_topic = "/ssf/thermalgrabber_ros/image_deg_celsius"
outputPath = "/home/seba/Desktop/Thermal"

bag = rosbag.Bag(bag_file)
info_dict = yaml.load(subprocess.Popen(['rosbag', 'info', '--yaml', bag_file], stdout=subprocess.PIPE).communicate()[0])
print(info_dict)


# print(bag)

genBag = bag.read_messages(read_topic)
tmax = 0
tmin = 0

for k,b in enumerate(genBag):
    print("OK, %d / %d" % (k, info_dict['messages']))

    cb = CvBridge()
    cv_image = cb.imgmsg_to_cv2(b.message, b.message.encoding)

    data = cv_image[:,:]
    if data.min() < tmin:
        tmin = data.min()

    if data.max() > tmax:
        tmax = data.max()

    print(tmax)
    print(tmin)

    # fig, ax = plt.subplots()
    # img = ax.imshow(data, cmap='magma')
    # fig.colorbar(img)
    # plt.savefig(outputPath + '/' + str(k) + '.jpg')
    # plt.close(fig)
