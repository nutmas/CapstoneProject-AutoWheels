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
    #im_list.append(msg)
    im_cv2 = bridge.imgmsg_to_cv2(msg, 'bgr8')
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
for i, bb_i in enumerate(bb_list):
    print('{:.2f}'.format(i * 100.0 / total_bb))
    # search for reference image
    im_ref = None
    for im_index, im_msg in enumerate(im_list):
        if bb_i.image_header.seq == im_msg[0].seq:
            im_ref = im_msg
            break
    if im_ref is None:
        print(bb_i.image_header.seq, 'not found')
        continue
    
    found_tl = False
    arc_full_fn = os.path.join('detection', 'im{:06}.jpg'.format(im_index))
    full_fn = os.path.join(detection_dir, 'im{:06}.jpg'.format(im_index))
    # if path already exists, image was already processed
    if os.path.exists(full_fn):
        continue
im_cv2 = im_list[im_index][1]
    for b_index, b in enumerate(bb_i.bounding_boxes):
        if b.Class != 'traffic light':
            continue
    
        found_tl = True
        cv2.rectangle(im_cv2, (b.xmin, b.ymin), (b.xmax,b.ymax), (255,0,0), 2)
        im_cv2_crop = im_cv2[b.ymin:b.ymax, b.xmin:b.xmax]

        arc_crop_fn = os.path.join('crops', 'im{:06}_crop{:02}.jpg'.format(im_index, b_index))
        crop_fn = os.path.join(crops_dir, 'im{:06}_crop{:02}.jpg'.format(im_index, b_index))
        plt.imsave(crop_fn, im_cv2_crop)
        myzip.write(crop_fn, arc_crop_fn)
    if found_tl:
        plt.imsave(full_fn, im_cv2)
        myzip.write(full_fn, arc_full_fn)
            
myzip.close()

# make video
print 'making video...'
# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'XVID')
video_out = cv2.VideoWriter(NAME + '.avi',fourcc, 20.0, (800,600))
im_percent = 100.0 / len(im_list)
for i, im in enumerate(im_list):
    print '{:.2f}%'.format(i * im_percent)
    video_out.write(im[1])
video_out.release()    

# clean temporary files
shutil.rmtree(temp_dir)

print 'finished'
