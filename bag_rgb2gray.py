#!/usr/bin/env python
# -*- coding: utf-8 -*
import sys, time
# numpy and scipy
import numpy as np
from scipy.ndimage import filters

# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage
import rosbag
import cv2
from cv_bridge import CvBridge
import glob
  

# invert bag
bag_rd = rosbag.Bag('/media/jz/sbq/Lidar_Camera/r3live_bag/r3live_opensource_bag/hku_campus_seq_00.bag', "r")
bag_data = bag_rd.read_messages()
bag_wt = rosbag.Bag('/media/jz/sbq/Lidar_Camera/r3live_bag/r3live_opensource_bag/hku_campus_seq_00_gray.bag', 'w')
window_name = 'Image'

bridge = CvBridge()

for topic, msg, t in bag_data:
    if topic == "/camera/image_color/compressed":
        print('/camera/image_color/compressed.header.stamp: {}'.format(t))
        np_arr = np.fromstring(msg.data, np.uint8)
        image_color = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        image_gray = cv2.cvtColor(image_color, cv2.COLOR_BGR2GRAY)
        gray_msg = bridge.cv2_to_imgmsg(image_gray, encoding="mono8")
        gray_msg.header.stamp = t

        bag_wt.write('/camera/image_color', gray_msg, t)
    
    elif topic == "/livox/imu":
        print('/livox/imu.header.stamp: {}'.format(t))
        imu_msg = msg
        bag_wt.write('/livox/imu', imu_msg, t)
    
    elif topic == "/livox/lidar":
        print('/livox/lidar.header.stamp: {}'.format(t))
        lidar_msg = msg
        bag_wt.write('/livox/lidar', lidar_msg, t)


bag_wt.close()



