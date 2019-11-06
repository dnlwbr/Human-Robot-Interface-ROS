#!/usr/bin/env python
import cv2
import numpy as np
import rospy
import sys
# Ros Messages (cv_bridge does not support CompressedImage in python)
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray

from GazeMapper import GazeMapper
from GazeMapper import show_circle
from hri_udp_publisher.msg import Journal


class InstanceHelper:
    def __init__(self):
        self.mapper = GazeMapper()
        self.window_is_open = False
        self.journal = Journal()
        self.human_img = None
        self.human_img_sub = rospy.Subscriber("/EyeRecTooImage/compressed", CompressedImage, self.callback,
                                              queue_size=1, buff_size=2**20)
        rospy.loginfo("subscribed to /EyeRecTooImage/compressed")
        self.robot_img = cv2.imread('robot.jpg', cv2.IMREAD_ANYCOLOR)
        # self.robot_gaze_pub = rospy.Publisher('/hri_gaze_mapping/robot_gaze', Float32MultiArray, queue_size=10)
        self.human_gaze = []
        self.robot_gaze = []

    def callback(self, ros_data):
        human_arr = np.fromstring(ros_data.data, np.uint8)
        self.human_img = cv2.imdecode(human_arr, cv2.IMREAD_COLOR)
        self.journal = rospy.wait_for_message('hri_udp_publisher/gaze_journal', Journal)

    def gaze_preview(self, gp_human, gp_robot):
        field_preview = show_circle(self.human_img, gp_human, 20)
        robot_preview = show_circle(self.robot_img, gp_robot, 30, thickness=10)

        for hgp in self.human_gaze:
            field_preview = show_circle(field_preview, hgp, 20)
        for rgp in self.robot_gaze:
            robot_preview = show_circle(robot_preview, rgp, 30, thickness=10)

        field_preview = cv2.resize(field_preview, None, fx=0.5, fy=0.5)
        robot_preview = cv2.resize(robot_preview, None, fx=0.2, fy=0.2)
        cv2.imshow("Field view", field_preview)
        cv2.imshow("Robot with human gaze", robot_preview)
        self.window_is_open = True


def main():
    rospy.init_node('hri_gaze_mapping', disable_signals=True)
    instance = InstanceHelper()

    rospy.sleep(0.5)

    while not rospy.is_shutdown():
        try:
            x = int(round(float(instance.journal.data[2])))  # field.gaze.x
            y = int(round(float(instance.journal.data[3])))  # field.gaze.y
        except ValueError:
            continue

        ret = instance.mapper.update(instance.human_img, instance.robot_img)

        if ret:
            try:
                human_gaze, robot_gaze = instance.mapper.map((x, y))
            except TypeError:
                continue

            instance.gaze_preview(human_gaze, robot_gaze)

            key = cv2.waitKey(100) & 0xFF
            # SPACE is pressed
            if key == 32:
                instance.robot_gaze.append(robot_gaze)
                instance.human_gaze.append(human_gaze)
                print("Gaze point added")
            # BACKSPACE is pressed
            elif key == 8:
                instance.robot_gaze = []
                instance.human_gaze = []
                print("Reset selection")
            # ENTER is pressed
            elif key == 13:
                robot_view_gaze = instance.robot_img.copy()
                human_view_gaze = instance.human_img.copy()
                for gp in instance.robot_gaze:
                    robot_view_gaze = show_circle(robot_view_gaze, gp, 40, thickness=10)
                for gp in instance.human_gaze:
                    human_view_gaze = show_circle(human_view_gaze, gp, 10)
                cv2.imwrite('robot_gaze.jpg', robot_view_gaze)
                cv2.imwrite('human_gaze.jpg', human_view_gaze)

                # msg = Float32MultiArray()
                # msg.data = robot_gaze
                # rospy.loginfo("(x,y) gaze coordinate: " + str(msg.data))
                # self.robot_gaze_pub.publish(msg)
                cv2.destroyAllWindows()
                break
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
