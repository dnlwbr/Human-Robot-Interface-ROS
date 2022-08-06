#!/usr/bin/env

import rospy
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from pathlib import Path
from geometry_msgs.msg import PoseStamped, Transform


def get_list(target_cs=""):
    poses = []
    tf_list = sorted(Path("/home/weber/Pictures/object_data/stapler/stapler01/tf").glob('*_tf.yaml'))
    for tf_path in tf_list:
        pos = PoseStamped()

        with open(str(tf_path), 'r') as file:
            try:
                tf = yaml.safe_load(file)
            except yaml.YAMLError as err:
                print("Could not load yaml file")
                raise

        t = np.array([tf["translation"]["x"], tf["translation"]["y"], tf["translation"]["z"]])
        r = Rotation.from_quat([tf["rotation"]["x"], tf["rotation"]["y"], tf["rotation"]["z"], tf["rotation"]["w"]])

        # [right, down, forward] (OpenCV) ->
        if target_cs == "drb":  # [down, right, backwards]
            t = np.array([t[1], t[0], -t[2]])
            r_opencv = r.as_quat()
            r_drb = Rotation.from_quat([r_opencv[1], r_opencv[0], -r_opencv[2], r_opencv[3]])
            r = r_drb
        elif target_cs == "OpenGL":  # [right, up, backward]
            t = np.array([t[0], -t[1], -t[2]])
            r_opencv = r.as_quat()
            r_opengl = Rotation.from_quat([r_opencv[0], -r_opencv[1], -r_opencv[2], r_opencv[3]])
            r = r_opengl

        pos.header.frame_id = "map"
        pos.pose.position.x = t[0]
        pos.pose.position.y = t[1]
        pos.pose.position.z = t[2]
        pos.pose.orientation.x = r.as_quat()[0]
        pos.pose.orientation.y = r.as_quat()[1]
        pos.pose.orientation.z = r.as_quat()[2]
        pos.pose.orientation.w = r.as_quat()[3]
        poses.append(pos)

    return poses


def get_list_paper():
    # pose_bounds = np.load("/data/NovelViewSynthesis/nex-code/data/crest_demo/poses_bounds.npy")
    # pose_bounds = np.load("/data/datasets/nerf_llff_data/fern/poses_bounds.npy")
    pose_bounds = np.load("/data/datasets/nerf_real_360/vasedeck/poses_bounds.npy")
    poses = []
    for i in range(0, pose_bounds.shape[0]):
        pb = pose_bounds[i]
        pos = PoseStamped()

        # r = Rotation.from_matrix(np.array([[pb[0], pb[1], pb[2]],
        #                                    [pb[4], pb[5], pb[6]],
        #                                    [pb[8], pb[9], pb[10]]]))
        # t = np.array([pb[3], pb[7], pb[11]])    # (down, right, backwards)

        r = Rotation.from_matrix(np.array([[pb[0], pb[1], pb[2]],
                                           [pb[5], pb[6], pb[7]],
                                           [pb[10], pb[11], pb[12]]]))
        t = np.array([pb[3], pb[8], pb[13]])  # (down, right, backwards)

        pos.header.frame_id = "map"
        pos.pose.position.x = t[0]
        pos.pose.position.y = t[1]
        pos.pose.position.z = t[2]
        pos.pose.orientation.x = r.as_quat()[0]
        pos.pose.orientation.y = r.as_quat()[1]
        pos.pose.orientation.z = r.as_quat()[2]
        pos.pose.orientation.w = r.as_quat()[3]
        poses.append(pos)

    return poses


def publish():
    origin = PoseStamped()
    origin.header.frame_id = "map"
    pub_origin = rospy.Publisher('origin', PoseStamped, queue_size=1)
    pub1 = rospy.Publisher('cam0', PoseStamped, queue_size=1)
    pub2 = rospy.Publisher('cam050', PoseStamped, queue_size=1)
    pub3 = rospy.Publisher('cam100', PoseStamped, queue_size=1)
    pub4 = rospy.Publisher('cam150', PoseStamped, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(30)  # 10hz
    poses_list = get_list("")
    # poses_list = get_list_paper()
    rospy.loginfo("Publish")
    while not rospy.is_shutdown():
        pub_origin.publish(origin)
        # pub1.publish(poses_list[0])
        pub1.publish(poses_list[0])
        # pub2.publish(poses_list[1])
        pub2.publish(poses_list[50])
        # pub3.publish(poses_list[2])
        pub3.publish(poses_list[100])
        # pub4.publish(poses_list[3])
        pub4.publish(poses_list[150])
        rate.sleep()


if __name__ == '__main__':
    try:
        publish()
    except rospy.ROSInterruptException:
        pass
