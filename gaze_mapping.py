#!/usr/bin/env python
import cv2
import numpy as np
import rospy
import socket
import sys
# Ros Messages (cv_bridge does not support CompressedImage in python)
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray

from GazeMapper import GazeMapper
from GazeMapper import show_circle


class InstanceHelper:
    def __init__(self):
        self.mapper = GazeMapper()
        self.window_is_open = False
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("localhost", 2002))
        self.journal = None
        self.human_img = None
        self.human_img_sub = rospy.Subscriber("/EyeRecTooImage/compressed", CompressedImage, self.callback,
                                              queue_size=1, buff_size=2**20)
        rospy.loginfo("subscribed to /EyeRecTooImage/compressed")
        self.robot_img = cv2.imread('robot.jpg', cv2.IMREAD_ANYCOLOR)
        self.robot_gaze = None
        # self.robot_gaze_pub = rospy.Publisher('/hri_gaze_mapping/robot_gaze', Float32MultiArray, queue_size=10)

    def callback(self, ros_data):
        human_arr = np.fromstring(ros_data.data, np.uint8)
        self.human_img = cv2.imdecode(human_arr, cv2.IMREAD_COLOR)
        self.read_journal()

    def read_journal(self):
        data, _ = self.sock.recvfrom(512)  # buffer size is 512
        data = data.decode('UTF-8')
        data = data.split("\t")
        self.journal = data[:54]


def main():
    rospy.init_node('hri_gaze_mapping', disable_signals=True)
    instance = InstanceHelper()

    rospy.sleep(0.5)

    while not rospy.is_shutdown() and instance.robot_gaze is None:

        try:
            x = int(round(float(instance.journal[2])))  # field.gaze.x
            y = int(round(float(instance.journal[3])))  # field.gaze.y
        except ValueError:
            continue

        ret = instance.mapper.update(instance.human_img, instance.robot_img)

        if ret:
            try:
                human_gaze, robot_gaze = instance.mapper.map((x, y))
            except TypeError:
                continue
            instance.window_is_open = True
            key = cv2.waitKey(100) & 0xFF
            # ENTER or SPACE is pressed
            if key == 13 or key == 32:
                robot_view_gaze = show_circle(instance.robot_img, robot_gaze, 20)
                cv2.imwrite('robot_gaze.jpg', robot_view_gaze)

                msg = Float32MultiArray()
                msg.data = robot_gaze
                rospy.loginfo("(x,y) gaze coordinate: " + str(msg.data))
                instance.robot_gaze = robot_gaze
                # self.robot_gaze_pub.publish(msg)
                cv2.destroyAllWindows()
            # q or Esc is pressed
            elif key == ord('q') or key == 27:
                print("Abort.")
                sys.exit(0)
        else:
            if instance.window_is_open is True:
                cv2.destroyAllWindows()
                instance.window_is_open = False

    rospy.signal_shutdown("Gaze mapping finished")
    print("Shutting down ROS hri_gaze_mapping module")

    return instance.robot_gaze


if __name__ == '__main__':
    main()
