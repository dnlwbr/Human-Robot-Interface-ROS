import cv2
import numpy as np
import sys


class SelectiveSearch:

    def __init__(self, img=None, method="q", ignore=1, resize_scale=0.25):

        # Speed-up using multi threads
        # print(cv2.getNumThreads())
        # cv2.setNumThreads(4)

        self.img = img

        # Ignore boxes bigger than (value*100)% of the height/width
        self.ignore = ignore

        # Create resized copy of image
        self.resize_scale = resize_scale

        # Create Selective Search Segmentation Object using default parameters
        self.ss = cv2.ximgproc.segmentation.createSelectiveSearchSegmentation()

        self.update(img, method, ignore, resize_scale)

    def update(self, img, method="q", ignore=1, resize_scale=0.25):

        self.img = img
        self.ignore = ignore
        self.resize_scale = resize_scale
        img_resized = cv2.resize(img, None, fx=self.resize_scale, fy=self.resize_scale)

        # Set input image on which we will run segmentation
        self.ss.setBaseImage(img_resized)

        if method == 'f':
            # Switch to fast but low recall Selective Search method
            self.ss.switchToSelectiveSearchFast()
        elif method == 'q':
            # Switch to high recall but slow Selective Search method
            self.ss.switchToSelectiveSearchQuality()
        elif method == 's':
            # Switch to Single Strategy
            self.ss.switchToSingleStrategy()
        else:
            # If argument is neither f,q nor s print help message
            print("Valid methods are:")
            print("f = fast, q = quality, s = single")
            sys.exit(1)

    def process(self):

        # Run selective search segmentation on input image
        rects = self.ss.process()

        # Rescale rects
        rects = np.around(rects / self.resize_scale)
        rects = rects.astype(int)

        return rects

    def sort_out(self, rects, gazepoints):

        # Get image height/width
        img_height, img_width = self.img.shape[:2]

        # Analyse and sort out region proposals
        del_idx = []
        for i, rect in enumerate(rects):
            x, y, w, h = rect
            if not all(x <= gp[0] <= x + w and y <= gp[1] <= y + h
                       and w < img_width * self.ignore and h < img_height * self.ignore for gp in gazepoints):
                del_idx.append(i)
        rects_sorted = np.delete(rects, del_idx, axis=0)

        return rects_sorted
