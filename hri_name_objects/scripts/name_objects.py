#!/usr/bin/env python

import OnScreenDisplay
import AudioFeedback
import rospy
import tf2_ros
import tf2_geometry_msgs
from gb_visual_detection_3d_msgs.msg import BoundingBoxes3d
from geometry_msgs.msg import PointStamped


class InstanceHandler:

    def __init__(self):
        self.osd = OnScreenDisplay.OnScreenDisplay("Welcome.", False)
        self.audio_feedback = AudioFeedback.AudioFeedback()
        self.boxes = None
        self.gaze_point = None
        self.current_object = None
        self.last_object = None
        self.boundary = 0.02
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.transform = None

        topic = '/darknet_ros_3d/bounding_boxes'
        self.subscriber_boxes = rospy.Subscriber(topic, BoundingBoxes3d, self.callback_boxes, queue_size=1)
        rospy.loginfo(f"Subscribed to {topic}")

        topic = '/hololens2/gaze/hitpoint'
        self.subscriber_boxes = rospy.Subscriber(topic, PointStamped, self.callback_gaze, queue_size=1)
        rospy.loginfo(f"Subscribed to {topic}")

    def callback_boxes(self, data):
        if data.bounding_boxes is not None:
            if self.boxes is None:
                rospy.loginfo(f"Boxes initialized.")
            self.boxes = data

    def callback_gaze(self, data):
        if self.transform is None:
            try:
                self.transform = self.tf_buffer.lookup_transform("camera_base_leveled", # target frame
                                                                 data.header.frame_id,  # source frame
                                                                 rospy.Time(0),  # get latest tf
                                                                 rospy.Duration(1))  # wait for 1 second
                rospy.loginfo(f"Gaze point initialized.")
            #except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            except Exception as e:
                print(e)

        if self.transform is not None:
            self.gaze_point = tf2_geometry_msgs.do_transform_point(data, self.transform)

    def update_current_object(self):
        if self.boxes is None or self.gaze_point is None:
            return False

        self.current_object = None
        for box in self.boxes.bounding_boxes:
            if (box.xmin - self.boundary <= self.gaze_point.point.x <= box.xmax + self.boundary and
                    box.ymin - self.boundary <= self.gaze_point.point.y <= box.ymax + self.boundary and
                    box.zmin - self.boundary <= self.gaze_point.point.z <= box.zmax + self.boundary):
                self.current_object = box.Class
        return True

    def check_event(self):
        ret = self.update_current_object()
        if ret:
            if self.current_object != self.last_object:
                if self.current_object is not None:
                    self.feedback_on_focus_start()
                else:
                    self.feedback_on_focus_stop()
                self.last_object = self.current_object
                return True
        return False

    def feedback_on_focus_start(self):
        self.osd.change_text(f"You are looking at the {self.current_object}")
        self.audio_feedback.say(f"You are looking at the {self.current_object}")

    def feedback_on_focus_stop(self):
        self.osd.change_text("")


def main():
    rospy.init_node('name_objects')
    instance_handler = InstanceHandler()

    while not rospy.is_shutdown():
        instance_handler.check_event()


if __name__ == '__main__':
    main()
