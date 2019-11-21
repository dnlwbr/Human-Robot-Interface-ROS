#!/usr/bin/env python
import cv2
import numpy as np
import rospy
import sys
# Ros Messages (cv_bridge does not support CompressedImage in python)
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray

from GazeMapper import GazeMapper
from hri_udp_publisher.msg import Journal


def show_circle(img, gp, radius, color=(0, 255, 0), thickness=5):
    tmp = img.copy()
    cv2.circle(tmp, (gp[0], gp[1]), radius, color, thickness)
    return tmp


class InstanceHelper:
    def __init__(self):
        self.mapper = GazeMapper()
        self.journal = Journal()
        self.human_img = None
        self.human_img_sub = rospy.Subscriber("/EyeRecTooImage/compressed", CompressedImage, self.callback,
                                              queue_size=1, buff_size=2**20)
        rospy.loginfo("subscribed to /EyeRecTooImage/compressed")
        self.robot_img = cv2.imread('robot.jpg', cv2.IMREAD_ANYCOLOR)
        # self.robot_gaze_pub = rospy.Publisher('/hri_gaze_mapping/robot_gaze', Float32MultiArray, queue_size=10)
        self.robot_gaze = []
        self.human_gaze = []
        self.human_gaze_imgs = []

    def callback(self, ros_data):
        human_arr = np.fromstring(ros_data.data, np.uint8)
        self.human_img = cv2.imdecode(human_arr, cv2.IMREAD_COLOR)
        self.journal = rospy.wait_for_message('hri_udp_publisher/gaze_journal', Journal)

    def gaze_preview(self, gp_human, gp_robot=None):
        if gp_robot is None:
            field_preview = show_circle(self.human_img, gp_human, 20, color=(0, 0, 255))
            robot_preview = self.robot_img
        else:
            field_preview = show_circle(self.human_img, gp_human, 20)
            robot_preview = show_circle(self.robot_img, gp_robot, 40, thickness=10)

        font = cv2.FONT_HERSHEY_DUPLEX
        font_scale = 1
        margin = 10
        color = (0, 255, 0)
        thickness = 1

        text = f"Selected points: {len(self.human_gaze)}"
        text_size = cv2.getTextSize(text, font, font_scale, thickness)
        text_width = text_size[0][0]
        text_height = text_size[0][1]
        line_height = text_height + text_size[1] + margin

        x = self.human_img.shape[1] - margin - text_width
        y = margin + text_height + 0 * line_height

        # cv2.putText(field_preview, text, (x, y), font, font_scale, color, thickness)

        if self.mapper.img_ids is None:
            warning = "No markers detected"
            warning_width = (cv2.getTextSize(warning, font, font_scale, thickness))[0][0]
            x = self.human_img.shape[1] - margin - warning_width
            y = margin + text_height + 0 * line_height
            color = (0, 0, 255)
            cv2.putText(field_preview, warning, (x, y), font, font_scale, color, thickness)

        for rgp in self.robot_gaze:
            robot_preview = show_circle(robot_preview, rgp, 40, thickness=10)

        cv2.namedWindow('Field view', cv2.WINDOW_GUI_EXPANDED)
        cv2.imshow("Field view", field_preview)
        cv2.namedWindow('Robot with human gaze', cv2.WINDOW_GUI_EXPANDED)
        cv2.imshow("Robot with human gaze", robot_preview)


def main():
    rospy.init_node('hri_gaze_mapping', disable_signals=True)
    instance = InstanceHelper()

    rospy.sleep(0.5)

    while not rospy.is_shutdown():
        try:
            x = int(round(float(instance.journal.data[2])))  # field.gaze.x
            y = int(round(float(instance.journal.data[3])))  # field.gaze.y
            human_gaze = np.float32([x, y])
        except ValueError:
            continue

        ret = instance.mapper.update(instance.human_img, instance.robot_img)

        if ret:
            try:
                robot_gaze = instance.mapper.map(human_gaze)
            except TypeError:
                continue
        else:
            robot_gaze = None

        instance.gaze_preview(human_gaze, robot_gaze)

        key = cv2.waitKey(100) & 0xFF
        # SPACE is pressed
        if key == 32 and robot_gaze is not None:
            instance.robot_gaze.append(robot_gaze)
            instance.human_gaze.append(human_gaze)
            instance.human_gaze_imgs.append(instance.human_img)
            print(f"Gaze point added: {human_gaze}")
        # BACKSPACE is pressed
        elif key == 8:
            instance.robot_gaze = []
            instance.human_gaze = []
            instance.human_gaze_imgs = []
            print("Reset selection")
        # ENTER is pressed
        elif key == 13:
            for i, gp in enumerate(instance.human_gaze):
                human_view_gaze = instance.human_gaze_imgs[i]
                human_view_gaze = show_circle(human_view_gaze, gp, 20, thickness=3)
                cv2.imwrite(f'human_gaze{i}.jpg', human_view_gaze)
            robot_view_gaze = instance.robot_img.copy()
            for gp in instance.robot_gaze:
                robot_view_gaze = show_circle(robot_view_gaze, gp, 40, thickness=10)
            cv2.imwrite('robot_gaze.jpg', robot_view_gaze)

            # msg = Float32MultiArray()
            # msg.data = robot_gaze
            # rospy.loginfo("(x,y) gaze coordinate: " + str(msg.data))
            # self.robot_gaze_pub.publish(msg)
            break
        # q or Esc is pressed
        elif key == ord('q') or key == 27:
            print("Abort.")
            sys.exit(0)

    cv2.destroyAllWindows()

    rospy.signal_shutdown("Gaze mapping finished")
    print("Shutting down ROS hri_gaze_mapping module")

    return instance.robot_gaze


if __name__ == '__main__':
    main()
