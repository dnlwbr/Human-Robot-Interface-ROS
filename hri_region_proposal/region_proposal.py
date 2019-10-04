"""
Usage:
    python3 region_proposal.py input_image (f|q|s)
    f=fast, q=quality, s=single
Use "l" to display less rects, 'm' to display more rects, "q" to quit.
"""

import sys
import cv2


def main(img, gazepoint, method):
    # speed-up using multithreads
    cv2.setUseOptimized(True)
    cv2.setNumThreads(4)

    # Get image height/width
    img_height, img_width = img.shape[:2]

    # Ignore boxes bigger than 70% of the height/width
    ignore = 0.7

    # resize image
    resize_scale = 0.25
    img = cv2.resize(img, None, fx=resize_scale, fy=resize_scale)

    # create Selective Search Segmentation Object using default parameters
    ss = cv2.ximgproc.segmentation.createSelectiveSearchSegmentation()

    # set input image on which we will run segmentation
    ss.setBaseImage(img)

    # Switch to fast but low recall Selective Search method
    if method == 'f':
        ss.switchToSelectiveSearchFast()

    # Switch to high recall but slow Selective Search method
    elif method == 'q':
        ss.switchToSelectiveSearchQuality()

    # Switch to Single Strategy
    elif method == 's':
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
    # increment to increase/decrease total number reason proposals to be shown
    increment = 50

    # Marked box
    marked = 1

    while True:
        # Initialise number of found boxes
        found = 0

        # create a copy of original image
        imOut = img.copy()

        # iterate over all the region proposals
        for i, rect in enumerate(rects):
            # draw rectangle for region proposal till numShowRects
            if i < numShowRects:
                x, y, w, h = rect
                if x <= gazepoint[0]*resize_scale <= x + w and y <= gazepoint[1]*resize_scale <= y + h \
                        and w < img_width*resize_scale*ignore and h < img_height*resize_scale*ignore:
                    cv2.rectangle(imOut, (x, y), (x + w, y + h), (0, 255, 0), 1, cv2.LINE_AA)
                    found += 1
                    if found == marked:
                        cv2.rectangle(imOut, (x, y), (x + w, y + h), (255, 0, 0), 1, cv2.LINE_AA)
                        marked_rect = rect / resize_scale
            else:
                break

        print("Marked Region: " + str(marked_rect))

        # show output
        imOut = cv2.resize(imOut, None, fx=1/(2*resize_scale), fy=1/(2*resize_scale))
        cv2.imwrite('/home/weber/Videos/Test/tisch_marker/robot_gaze_box.jpg', imOut)
        cv2.imshow("Region proposal", imOut)

        # record key press
        key = cv2.waitKey(0) & 0xFF

        # b is pressed
        if key == ord('b'):
            # mark previous box
            if marked > 1:
                marked -= 1
        # n is pressed
        if key == ord('n'):
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
            break
        # q or Esc is pressed
        elif key == ord('q') or key == 27:
            print("Abort.")
            sys.exit(1)

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
