#!/usr/bin/env python

import rospy
import csv
import argparse
import cv2
import numpy as np
import message_filters
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge

from GazeMapper import GazeMapper


class InstanceHelper:
    def __init__(self, args):
        self.pathToFolder = args.pathToFolder
        self.bridge = CvBridge()
        self.mapper = GazeMapper()
        self.robot_image_sub = message_filters.Subscriber("/kinect2/sd/image_color_rect", Image)
        self.robot_info_sub = message_filters.Subscriber('/kinect2/sd/camera_info', CameraInfo)
        self.robot_gaze_pub = rospy.Publisher('/hri_gaze_mapping/robot_gaze', Float32MultiArray, queue_size=10)

    def callback(self, rgb_msg, camera_info):
        timestamps = readTimestamps(self.pathToFolder + '/FieldData.tsv')
        journal = readJournal(self.pathToFolder + '/JournalData.tsv')
        matched = match(timestamps, journal, 'sync.timestamp')
        human_view = cv2.VideoCapture(self.pathToFolder + '/Field.mp4')

        robot_image_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="rgb8")

        # Check if total number of frames equals number of timestamps
        assert human_view.get(cv2.CAP_PROP_FRAME_COUNT) == len(timestamps)
        # Starting frame if desired
        human_view.set(cv2.CAP_PROP_POS_FRAMES, len(timestamps) - 1)
        ret, frame = human_view.read()
        if not ret:
            exit(1)
        cur_frame_idx = int(human_view.get(cv2.CAP_PROP_POS_FRAMES)) - 1

        get = lambda name: int(round(np.median([float(entry[name]) for entry in matched[cur_frame_idx]['matches']])))
        x = get('field.gaze.x')
        y = get('field.gaze.y')

        self.mapper.update(robot_image_rgb, frame)
        human_gaze, robot_gaze = self.mapper.map((x, y))

        self.robot_gaze_pub.publish(robot_gaze)


def readTimestamps(filename):
    with open(filename, newline='\n') as file:
        reader = csv.DictReader(file, delimiter='\t')
        return [float(row['timestamp']) for row in reader]


def readJournal(filename):
    with open(filename, newline='\n') as file:
        reader = csv.DictReader(file, delimiter='\t')
        return [row for row in reader]


def match(timestamps, source, key_name='timestamp'):
    data = [{'timestamp': ts, 'matches': []} for ts in timestamps]

    current = 0
    lastIdx = len(data)-1
    for row in source:
        ts = float(row[key_name])

        # find first entry equal or larger than ts
        while ts > data[current]['timestamp'] and current < lastIdx:
            current += 1

        timediff = lambda idx: abs(ts - data[idx]['timestamp'])

        # Is current or prev closer to ts?
        prev = max(0, current-1)
        matchedIdx = prev if timediff(prev) < timediff(current) else current
        data[matchedIdx]['matches'].append(row)

    return data


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Overlay gaze on top of field video and map gaze on robot view')
    parser.add_argument('pathToFolder', action='store')
    args = parser.parse_args()

    rospy.init_node('hri_gaze_mapping')

    helper = InstanceHelper(args)

    ts = message_filters.ApproximateTimeSynchronizer([helper.robot_image_sub, helper.robot_info_sub], 10, 0.1)
    ts.registerCallback(helper.callback)

    try:
        # spin() keeps python from exiting until this node is stopped
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down gaze mapping")

    cv2.destroyAllWindows()
