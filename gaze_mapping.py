#!/usr/bin/env python
import cv2
import numpy as np
import rospy
import socket
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
        self.sock.bind(("", 2002))
        self.journal = None
        self.human_img_sub = rospy.Subscriber("/EyeRecTooImage/compressed", CompressedImage, self.callback,
                                              queue_size=1, buff_size=2**20)
        rospy.loginfo("subscribed to /EyeRecTooImage/compressed")
        # self.robot_gaze_pub = rospy.Publisher('/hri_gaze_mapping/robot_gaze', Float32MultiArray, queue_size=10)
        self.robot_gaze = None

    def callback(self, ros_data):
        human_arr = np.fromstring(ros_data.data, np.uint8)
        human_img = cv2.imdecode(human_arr, cv2.IMREAD_COLOR)
        self.read_journal()
        robot_filename = 'robot.jpg'
        robot_img = cv2.imread(robot_filename, cv2.IMREAD_ANYCOLOR)

        try:
            x = int(round(float(self.journal[2])))  # field.gaze.x
            y = int(round(float(self.journal[3])))  # field.gaze.y
        except ValueError:
            exit(1)

        ret = self.mapper.update(human_img, robot_img)
        if ret and self.robot_gaze is None:
            try:
                human_gaze, robot_gaze = self.mapper.map((x, y))
            except TypeError:
                exit(1)
            self.window_is_open = True
            key = cv2.waitKey(100) & 0xFF
            # ENTER or SPACE is pressed
            if key == 13 or key == 32:
                robot_view_gaze = show_circle(robot_img, robot_gaze, 20)
                cv2.imwrite('robot_gaze.jpg', robot_view_gaze)

                msg = Float32MultiArray()
                msg.data = robot_gaze
                rospy.loginfo("(x,y) gaze coordinate: " + str(msg.data))
                self.robot_gaze = robot_gaze
                # self.robot_gaze_pub.publish(msg)
        else:
            if self.window_is_open is True:
                cv2.destroyAllWindows()
                self.window_is_open = False

    def read_journal(self):
        data, _ = self.sock.recvfrom(512)  # buffer size is 512
        data = data.decode('UTF-8')
        data = data.split("\t")
        self.journal = data[:54]


def main():
    rospy.init_node('hri_gaze_mapping', disable_signals=True)
    instance = InstanceHelper()

    while not rospy.is_shutdown() and instance.robot_gaze is None:
        rospy.rostime.wallsleep(0.5)

    rospy.signal_shutdown("Gaze mapping finished")
    print("Shutting down ROS hri_gaze_mapping module")

    return instance.robot_gaze


if __name__ == '__main__':
    main()
