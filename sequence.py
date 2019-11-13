"""
Usage:
    python3 sequence.py input_path
"""

import argparse
import sys
import cv2

import gaze_mapping
import region_proposal
# import object_tracking


def main():
    # Get robot snapshot
    robot_filename = "robot.jpg"  # args.pathToFolder + '/robot.jpg'
    robot_view = cv2.imread(robot_filename, cv2.IMREAD_ANYCOLOR)

    # Gaze mapping
    robot_gaze = gaze_mapping.main()

    # Region proposal
    box = region_proposal.main(robot_view, "q", robot_gaze)  # , path=args.pathToFolder)

    # Object tracking
    # object_tracking.main(args, box)


if __name__ == '__main__':
    # If data path is not passed as command line arguments, quit and display help message
    # if len(sys.argv) < 2:
    #     print(__doc__)
    #     sys.exit(1)

    # Parse input argument
    # parser = argparse.ArgumentParser(description='Map gaze on robot view, propose ROI and track bounding box.')
    # parser.add_argument('pathToFolder', action='store')
    # parser.add_argument("-t", "--tracker", type=str, default="kcf", help="OpenCV object tracker type")
    # args = parser.parse_args()

    # main(args)
    main()
