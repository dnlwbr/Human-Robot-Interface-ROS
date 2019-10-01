#!/usr/bin/env python

"""
Usage:
    ./region_proposal.py input_image (f|q|s)
    f=fast, q=quality, s=single
Use "l" to display less rects, 'm' to display more rects, "q" to quit.
"""

import sys
import cv2


if __name__ == '__main__':
    # If image path and f/q is not passed as command
    # line arguments, quit and display help message
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)

    # speed-up using multithreads
    cv2.setUseOptimized(True)
    cv2.setNumThreads(4)

    # read image
    img = cv2.imread(sys.argv[1])
    img_height, img_width = img.shape[:2]

    # resize image
    # newHeight = 200
    # newWidth = int(img.shape[1] * 200 / img.shape[0])
    # img = cv2.resize(img, (newWidth, newHeight))
    resize_scale = 0.25
    img = cv2.resize(img, None, fx=resize_scale, fy=resize_scale)

    # create Selective Search Segmentation Object using default parameters
    ss = cv2.ximgproc.segmentation.createSelectiveSearchSegmentation()

    # set input image on which we will run segmentation
    ss.setBaseImage(img)

    # Switch to fast but low recall Selective Search method
    if sys.argv[2] == 'f':
        ss.switchToSelectiveSearchFast()

    # Switch to high recall but slow Selective Search method
    elif sys.argv[2] == 'q':
        ss.switchToSelectiveSearchQuality()

    # Switch to Single Strategy
    elif sys.argv[2] == 's':
        ss.switchToSingleStrategy()

    # if argument is neither f,q nor s print help message
    else:
        print(__doc__)
        sys.exit(1)

    # run selective search segmentation on input image
    rects = ss.process()
    print('Total Number of Region Proposals: {}'.format(len(rects)))

    # number of region proposals to show
    numShowRects = 100
    # increment to increase/decrease total number
    # of reason proposals to be shown
    increment = 50

    first = True

    while True:
        # create a copy of original image
        imOut = img.copy()

        # iterate over all the region proposals
        for i, rect in enumerate(rects):
            # draw rectangle for region proposal till numShowRects
            if i < numShowRects:
                x, y, w, h = rect
                if x <= 767*0.25 <= x + w and y <= 585*0.25 <= y + h \
                        and w < img_width*0.7*resize_scale and h < img_height*0.7*resize_scale:
                    cv2.rectangle(imOut, (x, y), (x + w, y + h), (0, 255, 0), 1, cv2.LINE_AA)
                    if first:
                        cv2.rectangle(imOut, (x, y), (x + w, y + h), (255, 0, 0), 1, cv2.LINE_AA)
                        first_rect = rect / resize_scale
                        first = False
            else:
                break

        print("First Region: " + str(first_rect))

        # show output
        imOut = cv2.resize(imOut, None, fx=1/(2*resize_scale), fy=1/(2*resize_scale))
        cv2.imwrite('/home/weber/Videos/Test/tisch_marker/robot_gaze_RP.jpg', imOut)
        cv2.imshow("Output", imOut)

        # record key press
        key = cv2.waitKey(0) & 0xFF

        # m is pressed
        if key == 109:
            # increase total number of rectangles to show by increment
            numShowRects += increment
        # l is pressed
        elif key == 108 and numShowRects > increment:
            # decrease total number of rectangles to show by increment
            numShowRects -= increment
        # q is pressed
        elif key == 113:
            break
    # close image show window
    cv2.destroyAllWindows()
