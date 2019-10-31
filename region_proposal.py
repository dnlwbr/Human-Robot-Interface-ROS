#!/usr/bin/env python
"""
Usage:
    python3 region_proposal.py input_image (f|q|s)
    f=fast, q=quality, s=single
Use "l" to display less rects, 'm' to display more rects, "q" to quit.
"""

import argparse
import cv2
import numpy as np
import sys

from GazeMapper import show_circle


def main(img, gazepoint, method, path='.'):
    # speed-up using multithreads
    cv2.setUseOptimized(True)
    cv2.setNumThreads(4)

    # Ignore boxes bigger than 70% of the height/width
    ignore = 0.7

    # create resized copy of image
    resize_scale = 0.25
    img_resized = cv2.resize(img, None, fx=resize_scale, fy=resize_scale)

    # Get resized image height/width
    img_resized_height, img_resized_width = img_resized.shape[:2]

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

    # number of region proposals to show
    numShowRects = 100
    # increment to increase/decrease total number reason proposals to be shown
    increment = 50

    # Marked box
    marked = 1

    # Show not only marked bounding box
    hide_unmarked = False

    while True:
        # Initialise number of found boxes
        found = 0

        # create a copies of original image
        img_preview = img_resized.copy()
        img_out = img.copy()

        # check if marked region exists in order to display it on top level
        marked_rect = None
        marked_rect_exists = False

        # iterate over all the region proposals
        for i, rect in enumerate(rects):
            # draw rectangle for region proposal till numShowRects
            if i < numShowRects:
                x, y, w, h = rect
                if x <= gazepoint[0]*resize_scale <= x + w and y <= gazepoint[1]*resize_scale <= y + h \
                        and w < img_resized_width*ignore and h < img_resized_height*ignore:
                    if hide_unmarked is False:
                        cv2.rectangle(img_preview, (x, y), (x + w, y + h), (0, 255, 0), 1, cv2.LINE_AA)
                    found += 1
                    if found == marked:
                        marked_rect = rect
                        marked_rect_exists = True
            else:
                break

        if marked_rect_exists is True:
            cv2.rectangle(img_preview, (marked_rect[0], marked_rect[1]),
                          (marked_rect[0] + marked_rect[2], marked_rect[1] + marked_rect[3]), (255, 0, 0), 1,
                          cv2.LINE_AA)
            marked_rect = np.around(marked_rect / resize_scale)
            marked_rect = marked_rect.astype(int)
            print(f"Marked region {marked}/{found}: {marked_rect}")

        # show preview
        img_preview = cv2.resize(img_preview, None, fx=1/(2*resize_scale), fy=1/(2*resize_scale))
        img_preview = show_circle(img_preview, gazepoint/2, 10)
        cv2.imshow("Region proposal", img_preview)

        # save image with chosen box
        img_out = show_circle(img_out, gazepoint, 20)
        cv2.rectangle(img_out, (marked_rect[0], marked_rect[1]),
                      (marked_rect[0] + marked_rect[2], marked_rect[1] + marked_rect[3]), (255, 0, 0), 2, cv2.LINE_AA)
        cv2.imwrite(path + '/robot_gaze_box.jpg', img_out)

        # record key press
        key = cv2.waitKey(0) & 0xFF

        # h is pressed
        if key == ord('h'):
            # hide/unhide unmarked boxes
            hide_unmarked = not hide_unmarked
        # b is pressed
        elif key == ord('b'):
            # mark previous box
            if marked > 1:
                marked -= 1
        # n is pressed
        elif key == ord('n'):
            # mark next box
            if marked < found:
                marked += 1
        # m is pressed
        elif key == ord('m'):
            # increase total number of rectangles to show by increment
            numShowRects += increment
        # l is pressed
        elif key == ord('l') and numShowRects > increment:
            # decrease total number of rectangles to show by increment
            numShowRects -= increment
        # ENTER or SPACE is pressed
        elif key == 13 or key == 32:
            # Finish with current values
            print(f"--> Finally selected: {marked_rect}")
            break
        # q or Esc is pressed
        elif key == ord('q') or key == 27:
            print("Abort.")
            sys.exit(0)

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

    gazepoint = [763.93665, 576.1394]

    main(input_image, gazepoint, method)
