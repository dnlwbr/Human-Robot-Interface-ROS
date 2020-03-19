#!/usr/bin/env python
import cv2
import numpy as np
import rospy
import sys
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage

from GazeMapper import GazeMapperFeature as GazeMapper
from hri_gaze_mapping.msg import Gaze
from hri_udp_publisher.msg import Journal


def show_circle(img, gp, radius, color=(0, 255, 0), thickness=5):
    tmp = img.copy()
    cv2.circle(tmp, (gp[0], gp[1]), radius, color, thickness)
    return tmp


class InstanceHelper:
    def __init__(self):
        self.bridge = CvBridge()
        self.mapper = GazeMapper()
        self.journal = Journal()
        self.journal_dict = dict()
        self.robot_gaze = []
        self.human_gaze = []
        self.human_gaze_imgs = []
        self.human_img = None
        self.human_img_sub = rospy.Subscriber("/EyeRecTooImage/compressed", CompressedImage, self.callback_human,
                                              queue_size=1, buff_size=2 ** 20)
        rospy.loginfo("Subscribed to /EyeRecTooImage/compressed")
        self.robot_img = None
        self.robot_img_sub = rospy.Subscriber("/kinect2/hd/image_color/compressed", CompressedImage, self.callback,
                                              queue_size=1, buff_size=2 ** 20)
        rospy.loginfo("Subscribed to /kinect2/hd/image_color/compressed")
        self.robot_gaze_pub = rospy.Publisher('/hri_gaze_mapping/robot_gaze', Gaze, queue_size=1)
        rospy.loginfo("Publishing robot gaze to /hri_gaze_mapping/robot_gaze")

    def callback_human(self, human_img):
        try:
            self.human_img = self.bridge.compressed_imgmsg_to_cv2(human_img, desired_encoding="passthrough")
        except CvBridgeError as e:
            print(e)

    def callback(self, robot_img):
        try:
            self.robot_img = self.bridge.compressed_imgmsg_to_cv2(robot_img, desired_encoding="passthrough")
        except CvBridgeError as e:
            print(e)

        self.journal = rospy.wait_for_message('hri_udp_publisher/gaze_journal', Journal)
        keys = self.journal.keys
        values = self.journal.values
        self.journal_dict = dict(zip(keys, values))

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

        # x = self.human_img.shape[1] - margin - text_width
        # y = margin + text_height + 0 * line_height
        # cv2.putText(field_preview, text, (x, y), font, font_scale, color, thickness)

        if self.mapper.__class__.__name__ == "GazeMapperAruco":
            self.mapper.print_warning(field_preview, robot_preview, font, font_scale, thickness, margin, text_height,
                                      line_height)

        for rgp in self.robot_gaze:
            # TODO: Problem if Robot moves after gaze point is set
            robot_preview = show_circle(robot_preview, rgp, 40, thickness=10)

        cv2.namedWindow('Field view', cv2.WINDOW_GUI_EXPANDED)
        cv2.imshow("Field view", field_preview)
        cv2.namedWindow('Robot with human gaze', cv2.WINDOW_GUI_EXPANDED)
        cv2.imshow("Robot with human gaze", robot_preview)


def main():
    rospy.init_node('hri_gaze_mapping', disable_signals=True)
    instance = InstanceHelper()
    robot_gaze_msg = Gaze()

    rospy.sleep(1.5)

    assert instance.journal_dict != {}, "Gaze is published?"

    while not rospy.is_shutdown():
        x = int(round(float(instance.journal_dict["field.gaze.x"])))
        y = int(round(float(instance.journal_dict["field.gaze.y"])))
        human_gaze = np.float32([x, y])

        ret = instance.mapper.update(instance.human_img, instance.robot_img, human_gaze)

        if ret:
            robot_gaze = instance.mapper.map(human_gaze)
            robot_gaze_msg.x, robot_gaze_msg.y = robot_gaze
            instance.robot_gaze_pub.publish(robot_gaze_msg)
        else:
            robot_gaze = None

        instance.gaze_preview(human_gaze, robot_gaze)

        key = cv2.waitKey(2) & 0xFF
        # PRESENTER1 is pressed
        if key == 86:
            # Freeze preview
            while True:
                key = cv2.waitKey(100) & 0xFF
                if key == 86:
                    break
        # SPACE or PRESENTER2 is pressed
        elif (key == 32 or key == 85) and robot_gaze is not None:
            instance.robot_gaze.append(robot_gaze)
            instance.human_gaze.append(human_gaze)
            instance.human_gaze_imgs.append(instance.human_img)
            print("Gaze point added:")
            print(f"\tHuman: {human_gaze}")
            print(f"\tRobot: {robot_gaze}")
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
            break
        # q or Esc is pressed
        elif key == ord('q') or key == 27:
            print("Abort.")
            sys.exit(0)

    cv2.destroyAllWindows()

    rospy.loginfo("Shutting down ROS hri_gaze_mapping module")
    rospy.signal_shutdown("Gaze mapping finished")

    return instance.robot_gaze


if __name__ == '__main__':
    main()
