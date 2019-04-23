import rosbag
import sys
import cv2
import os.path
from cv_bridge import CvBridge


bag = rosbag.Bag(sys.argv[1])
out_dir = sys.argv[2]
i = 0
bridge = CvBridge()
for topic, im, t in bag.read_messages(topics=['/image_raw']):
    print 'extrating ', i
    cv_image = bridge.imgmsg_to_cv2(im, "bgr8")
    cv2.imwrite(os.path.join(out_dir, '{:05d}.jpg'.format(i)), cv_image)
    i += 1
bag.close()
