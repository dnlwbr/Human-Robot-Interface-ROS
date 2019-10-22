#!/usr/bin/env python

import rospy
import cv2
from imutils.video import FPS
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge


class InstanceHelper:
    def __init__(self):
        self.bridge = CvBridge()
        self.robot_image_sub = rospy.Subscriber("/kinect2/sd/image_color_rect", Image, self.callback)
        self.region_proposal = None
        self.rgb_msg = None
        self.img_cv2 = None

    def callback(self, rgb_msg):
        self.rgb_msg = rgb_msg

    def listen(self):
        self.region_proposal = rospy.wait_for_message('/hri_region_proposal/region_proposal', Float32MultiArray)
        self.img_cv2 = helper.bridge.imgmsg_to_cv2(self.rgb_msg, desired_encoding="rgb8")


def main(helper, tracker_name="kcf"):
    initBB = helper.region_proposal

    # extract the OpenCV version info
    (major, minor) = cv2.__version__.split(".")[:2]

    # if we are using OpenCV 3.2 OR BEFORE, we can use a special factory
    # function to create our object tracker
    if int(major) == 3 and int(minor) < 3:
        tracker = cv2.Tracker_create(tracker_name.upper())

    # otherwise, for OpenCV 3.3 OR NEWER, we need to explicity call the
    # approrpiate object tracker constructor:
    else:
        # initialize a dictionary that maps strings to their corresponding
        # OpenCV object tracker implementations
        OPENCV_OBJECT_TRACKERS = {
            "csrt": cv2.TrackerCSRT_create,
            "kcf": cv2.TrackerKCF_create,
            "boosting": cv2.TrackerBoosting_create,
            "mil": cv2.TrackerMIL_create,
            "tld": cv2.TrackerTLD_create,
            "medianflow": cv2.TrackerMedianFlow_create,
            "mosse": cv2.TrackerMOSSE_create
        }

        # grab the appropriate object tracker using our dictionary of
        # OpenCV object tracker objects
        tracker = OPENCV_OBJECT_TRACKERS[tracker_name]()

    # initialize the FPS throughput estimator
    fps = None

    # Check if initialisation of tracker is necessary because of passed initBB
    if initBB is not None:
        initTracker = True
    else:
        initTracker = False

    # loop over frames from the video stream
    while True:
        frame = helper.img_cv2
        # resize the frame (so we can process it faster), grab the frame dimensions and adjust bounding box
        resize_scale = 0.25
        frame = cv2.resize(frame, None, fx=resize_scale, fy=resize_scale)
        (H, W) = frame.shape[:2]
        initBB = initBB * resize_scale

        # check to see if we are currently tracking an object
        if initBB is not None:
            # If bounding box is initially passed to the main function, initialize tracker at the first pass of the loop
            if initTracker is True:
                # start OpenCV object tracker using the passed bounding box
                # coordinates, then start the FPS throughput estimator as well
                tracker.init(frame, initBB)
                fps = FPS().start()
                initTracker = False

            # grab the new bounding box coordinates of the object
            (success, box) = tracker.update(frame)

            # check to see if the tracking was a success
            if success:
                (x, y, w, h) = [int(v) for v in box]
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 1)

            # update the FPS counter
            fps.update()
            fps.stop()

            # initialize the set of information we'll be displaying on
            # the frame
            info = [
                ("Tracker", tracker_name),
                ("Success", "Yes" if success else "No"),
                ("FPS", "{:.2f}".format(fps.fps())),
            ]

            # loop over the info tuples and draw them on our frame
            for (i, (k, v)) in enumerate(info):
                text = "{}: {}".format(k, v)
                cv2.putText(frame, text, (10, H - ((i * 20) + 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 1)

        # show the output frame
        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1) & 0xFF

        # if the 's' key is selected, we are going to "select" a bounding box to track
        if key == ord("s"):
            # select the bounding box of the object we want to track (make
            # sure you press ENTER or SPACE after selecting the ROI)
            initBB = cv2.selectROI("Frame", frame, fromCenter=False, showCrosshair=True)

            # start OpenCV object tracker using the supplied bounding box
            # coordinates, then start the FPS throughput estimator as well
            tracker.init(frame, initBB)
            fps = FPS().start()

        # if the q or Esc key was pressed, break from the loop
        elif key == ord('q') or key == 27:
            break

    # close all windows
    cv2.destroyAllWindows()


if __name__ == '__main__':

    rospy.init_node('hri_object_tracking')
    helper = InstanceHelper()

    while not rospy.is_shutdown():
        rospy.rostime.wallsleep(0.5)

        # record key press
        key = cv2.waitKey(30) & 0xFF

        # ENTER or SPACE is pressed
        if key == 13 or key == 32:
            helper.listen()
            main(helper, tracker_name="kcf")


