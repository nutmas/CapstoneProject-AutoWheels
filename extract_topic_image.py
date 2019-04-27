import tempfile
import zipfile
import os
import shutil

import cv2
import rosbag
import cv_bridge
import rosmsg
import matplotlib.pyplot as plt
import sys

BAG_PATH = sys.argv[1]
IMAGE_TOPIC = sys.argv[2]
NAME = sys.argv[3]

bridge = cv_bridge.CvBridge()
bag = rosbag.Bag(BAG_PATH)

bb = []
for topic, msg, t in bag.read_messages(topics=['/darknet_ros/bounding_boxes']):
    bb.append(msg)
im_list = []
for topic, msg, t in bag.read_messages(topics=[IMAGE_TOPIC]):
    im_cv2 = bridge.imgmsg_to_cv2(msg, msg.encoding)
    im_list.append([msg.header, im_cv2])

bag.close()

bb_list = bb
name = NAME


out_fn = name + '.zip'
if os.path.exists(out_fn):
   os.remove(out_fn)
temp_dir = tempfile.mkdtemp()
detection_dir = os.path.join(temp_dir, 'detection')
crops_dir = os.path.join(temp_dir, 'crops')
os.mkdir(detection_dir)
os.mkdir(crops_dir)
myzip = zipfile.ZipFile(out_fn, 'w')
total_bb = len(bb_list)

im_percent = 100.0 / len(im_list)
for i, im in enumerate(im_list):
    print '{:.2f}%'.format(i * im_percent)
    im_fn =  '{:06}.jpg'.format(i)
    full_fn = os.path.join(temp_dir, im_fn)
    plt.imsave(full_fn, im[1])
    myzip.write(full_fn, im_fn)
myzip.close()

# clean temporary files
shutil.rmtree(temp_dir)

print 'finished'
