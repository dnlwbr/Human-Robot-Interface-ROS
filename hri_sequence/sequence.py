"""
Usage:
    python3 sequence.py input_path
"""

import argparse
import sys
import cv2

sys.path.append('/home/weber/Documents/Research/workspaces/hri_ws/src/hri_gaze_mapping')
import gaze_mapping
sys.path.append('/home/weber/Documents/Research/workspaces/hri_ws/src/hri_region_proposal')
import region_proposal
sys.path.append('/home/weber/Documents/Research/workspaces/hri_ws/src/hri_object_tracking')
import object_tracking


def main(args):
    # Get robot snapshot
    # cam = cv2.VideoCapture(0)
    robot_filename = args.pathToFolder + '/robot.jpg'
    frame = cv2.imread(robot_filename, cv2.IMREAD_ANYCOLOR)
    #

    while True:
        # ret, frame = cam.read()
        #
        # if not ret:
        #     break

        # Gaze mapping
        human_gaze, robot_gaze = gaze_mapping.main(args, frame)

        key = cv2.waitKey(200) & 0xFF

        # Spacebar is pressed
        if key == 32:
            snapshot_robot = frame
            cv2.imwrite("snapshot_robot.jpg", frame)
            print("Snapshot taken.")
            break
        # q or Esc is pressed
        elif key == ord('q') or key == 27:
            print("Abort.")
            sys.exit(1)

    # cam.release()
    cv2.destroyAllWindows()

    # Region proposal
    box = region_proposal.main(frame, robot_gaze, "s")

    # Object tracking
    # object_tracking.main()
    # object_tracking.py [-h] [-v VIDEO] [-t TRACKER] [-b BOX]


if __name__ == '__main__':
    # If data path is not passed as command line arguments, quit and display help message
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    # Parse input argument
    parser = argparse.ArgumentParser(description='Overlay gaze on top of field video and map gaze on robot view')
    parser.add_argument('pathToFolder', action='store')
    args = parser.parse_args()

    main(args)
