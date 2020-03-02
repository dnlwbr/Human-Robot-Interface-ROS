#!/usr/bin/env python
"""
Usage:
    python3 region_proposal.py input_image (f|q|s)
    f=fast, q=quality, s=single
Use "l" to display less rects, 'm' to display more rects, "q" to quit.
"""

import cv2
import numpy as np
import sys

from gaze_mapping import show_circle


def main(img, method, gazepoints=None, path='.'):
    # speed-up using multi threads
    # print(cv2.getNumThreads())
    # cv2.setNumThreads(4)

    if gazepoints is None:
        gazepoints = []

    # Ignore boxes bigger than (value*100)% of the height/width
    ignore = 1

    # create resized copy of image
    resize_scale = 0.25
    img_resized = cv2.resize(img, None, fx=resize_scale, fy=resize_scale)

    # create Selective Search Segmentation Object using default parameters
    ss = cv2.ximgproc.segmentation.createSelectiveSearchSegmentation()

    # set input image on which we will run segmentation
    ss.setBaseImage(img_resized)

    if method == 'f':
        # Switch to fast but low recall Selective Search method
        ss.switchToSelectiveSearchFast()
    elif method == 'q':
        # Switch to high recall but slow Selective Search method
        ss.switchToSelectiveSearchQuality()
    elif method == 's':
        # Switch to Single Strategy
        ss.switchToSingleStrategy()

    # if argument is neither f,q nor s print help message
    else:
        print(__doc__)
        sys.exit(1)

    # run selective search segmentation on input image
    rects = ss.process()

    # rescale rects
    rects = np.around(rects / resize_scale)
    rects = rects.astype(int)

    # Get image height/width
    img_height, img_width = img.shape[:2]

    # analyse and sort out region proposals
    del_idx = []
    for i, rect in enumerate(rects):
        x, y, w, h = rect
        if not all(x <= gp[0] <= x + w and y <= gp[1] <= y + h
                   and w < img_width * ignore and h < img_height * ignore for gp in gazepoints):
            del_idx.append(i)
    rects_sorted = np.delete(rects, del_idx, axis=0)
    rects = rects_sorted

    ################################
    # # Save proposals if necessary
    # import pickle
    # file = open('rects.pkl', 'wb')
    # pickle.dump(rects, file)
    # file.close()
    # # Load proposals if necessary
    # import pickle
    # file = open('rects.pkl', 'rb')
    # rects = pickle.load(file)
    ################################

    # number of region proposals to show
    num_show_rects = 1 if len(gazepoints) > 0 else 100
    # increment to increase/decrease total number reason proposals to be shown
    increment = 1 if len(gazepoints) > 0 else 50

    # Marked box
    marked = 1

    # Toggle unmarked boxes and gaze points
    hide_unmarked = False
    hide_gp = False

    # Print Info only if something changed
    action_toggle = True

    while True:
        # Initialise counter for drawn boxes
        drawn = 0

        # create a copies of original image
        img_out = img.copy()

        # check if marked region exists in order to display it on top level
        marked_rect = None
        marked_rect_exists = False

        # draw gaze points
        if hide_gp is False:
            for gp in gazepoints:
                img_out = show_circle(img_out, gp, 40, thickness=10)

        # iterate over all the region proposals
        for i, rect in enumerate(rects):
            # draw rectangle for region proposal till num_show_rects
            if i < num_show_rects:
                x, y, w, h = rect
                if hide_unmarked is False:
                    cv2.rectangle(img_out, (x, y), (x + w, y + h), (0, 255, 0), 2, cv2.LINE_AA)
                drawn += 1
                if drawn == marked:
                    marked_rect = np.array([x, y, x + w, y + h])
                    marked_rect_exists = True
            else:
                break

        if marked_rect_exists is True:
            if action_toggle:
                print(f"Marked region {marked}/{num_show_rects}({len(rects)}): {marked_rect}")
            cv2.rectangle(img_out, (marked_rect[0], marked_rect[1]), (marked_rect[2], marked_rect[3]),
                          (255, 0, 0), 2, cv2.LINE_AA)

        # show preview
        cv2.namedWindow('Region proposal', cv2.WINDOW_GUI_EXPANDED)
        cv2.imshow("Region proposal", img_out)

        # record key press
        key = cv2.waitKey(0) & 0xFF

        # g is pressed
        if key == ord('g'):
            # hide/unhide gaze points
            hide_gp = not hide_gp
        # h is pressed
        elif key == ord('h'):
            # hide/unhide unmarked boxes
            hide_unmarked = not hide_unmarked
        # b is pressed
        elif key == ord('b') and marked > 1:
            # mark previous box
            marked -= 1
            action_toggle = True
        # n is pressed
        elif key == ord('n') and marked < drawn:
            # mark next box
            marked += 1
            action_toggle = True
        # m is pressed
        elif key == ord('m') and num_show_rects < len(rects):
            # increase total number of rectangles to show by increment
            num_show_rects += increment
            action_toggle = True
        # l is pressed
        elif key == ord('l') and num_show_rects > marked:
            # decrease total number of rectangles to show by increment
            num_show_rects -= increment
            action_toggle = True
        # ENTER or SPACE is pressed
        elif key == 13 or key == 32:
            # save image
            cv2.imwrite(path + '/robot_gaze_box.jpg', img_out)
            # Finish with current values
            print(f"--> Finally selected: {marked_rect}")
            break
        # q or Esc is pressed
        elif key == ord('q') or key == 27:
            print("Abort.")
            sys.exit(0)
        else:
            action_toggle = False

    # close image show window
    cv2.destroyAllWindows()

    return marked_rect


if __name__ == '__main__':
    # If image path and f/q/s is not passed as command line arguments, quit and display help message
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)

    # read image
    input_image = cv2.imread(sys.argv[1])

    # read method
    method = sys.argv[2]

    main(input_image, method)
