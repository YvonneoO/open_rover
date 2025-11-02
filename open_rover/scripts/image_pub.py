#!/usr/bin/env python3
import numpy as np
import os
import cv2
import json
import time
import argparse
import subprocess
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


# argparse setting
timestr = time.strftime('%Y-%m-%d-%H-%M')
parser = argparse.ArgumentParser(description='Segment Anything Model 2')
parser.add_argument('-i', '--input', help='Camera Number', type=int, default=0)
parser.add_argument('--show', action='store_true', help='show opencv window (needs X)')
# parser.add_argument('-sp', '--save-path', help='Path to input image.', default='/data/capture/'+timestr)
#parser.add_argument('-sp', '--save-path', help='Path to input image.', default='/var/tmp/'+timestr)

# important: rosrun adds its own args, so we ignore unknown
args, _ = parser.parse_known_args()

# os.makedirs(args.save_path + "/360", exist_ok=True)

# Initialize ROS node and publisher
rospy.init_node('image_publisher', anonymous=True)
image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
camera_info_pub = rospy.Publisher('/camera/camera_info', CameraInfo, queue_size=10)
bridge = CvBridge()

if args.show:
    cv2.namedWindow('video', cv2.WINDOW_NORMAL)

cap = cv2.VideoCapture(args.input)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

if (cap.isOpened() == False):
    rospy.logerr(f"Failed to open camera at index {args.input}")
    exit(1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2880)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1440)

# Read actual camera properties
actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = cap.get(cv2.CAP_PROP_FPS)
backend = cap.getBackendName()

print(f"Camera opened successfully:")
print(f"  Device: /dev/video{args.input}")
print(f"  Backend: {backend}")
print(f"  Resolution: {actual_width}x{actual_height}")
print(f"  FPS: {fps}")
print("Start capturing...")

# Create CameraInfo message
camera_info = CameraInfo()
camera_info.header.frame_id = "camera_frame"
camera_info.width = actual_width
camera_info.height = actual_height
# Note: Camera intrinsics (K, D, R, P) should be set from calibration


capture_count = 0

while not rospy.is_shutdown():
  # Publish image and camera info to ROS topics
  try:
    ret, frame = cap.read()
    if not ret:
      rospy.logerr("Failed to read frame from camera")
      continue

    # Set timestamp
    timestamp = rospy.Time.now()

    # Publish image
    ros_image = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
    ros_image.header.stamp = timestamp
    ros_image.header.frame_id = "camera_frame"
    image_pub.publish(ros_image)

    # Publish camera info
    camera_info.header.stamp = timestamp
    camera_info_pub.publish(camera_info)
  except Exception as e:
    rospy.logerr(f"Failed to publish image/camera_info: {e}")

  if args.show:
    cv2.imshow('video', frame)
    # cv2.imwrite(args.save_path + "/360/" + format(capture_count, '04d') + ".jpg", frame)

    # data = {
    #   "frame_count": capture_count
    # }

    # with open(args.save_path + "/frame_count.json", "w") as json_file:
    #   json.dump(data, json_file)

    key = cv2.waitKey(1)
    if key == 27:
      break

  capture_count += 1

cap.release()

if args.show:
  cv2.destroyAllWindows()
