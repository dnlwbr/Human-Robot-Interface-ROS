#!/usr/bin/env python


import sys
import rospy
from hri_cloud_segmentation.srv import *


def call_segmentation(gp_x, gp_y, gp_z, r):
    msg = SegmentRequest()
    msg.gazeHitPoint.point.x = gp_x
    msg.gazeHitPoint.point.y = gp_y
    msg.gazeHitPoint.point.z = gp_z
    msg.radius = r
    rospy.wait_for_service('hri_cloud_segmentation/Segment')
    try:
        seg = rospy.ServiceProxy('hri_cloud_segmentation/Segment', Segment)
        resp = seg(msg)
        return resp.success
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def usage():
    return "%s GazePoint Radius" % sys.argv[0]


if __name__ == "__main__":
    if len(sys.argv) == 5:
        gp_x = float(sys.argv[1])
        gp_y = float(sys.argv[2])
        gp_z = float(sys.argv[3])
        r = float(sys.argv[4])
    else:
        print(usage())
        sys.exit(1)
    print("Requesting segmentation for g=(%s,%s,%s) and radius=%s" % (gp_x, gp_y, gp_z, r))

    if call_segmentation(gp_x, gp_y, gp_z, r):
        print("Success")
    else:
        print("Fail")
