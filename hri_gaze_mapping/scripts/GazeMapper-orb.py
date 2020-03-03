import cv2
import numpy as np


class GazeMapperOrb:
    def __init__(self):
        self.ref, self.img = None, None
        self.ref_dimensions, self.img_dimensions = None, None
        self.orb = cv2.ORB_create()  # Initiate ORB detector
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)  # create BFMatcher object
        self.GOOD_MATCH_PERCENT = 0.15
        self.img_kp, self.img_des = None, None
        self.ref_kp, self.ref_des = None, None
        self.match_img = None

    def update(self, img, ref_img):
        self.img = img
        self.ref = ref_img
        self.img_dimensions = self.img.shape
        self.ref_dimensions = self.ref.shape
        self.img_kp, self.img_des = self.detect(self.img)
        self.ref_kp, self.ref_des = self.detect(self.ref)

        # self.ref = cv2.resize(ref_img, (self.img_dimensions[1], self.img_dimensions[0]))

        if None not in [self.img_kp, self.ref_kp]:
            return True
        else:
            return False

    def detect(self, img):
        if np.shape(img)[2] == 3:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        else:
            gray = img

        # return the keypoints and descriptors found with ORB
        return self.orb.detectAndCompute(gray, None)

    def map(self, gazepoint):

        # x_crop = 50#np.shape(img)[1] * 0.2
        # y_crop = 50#np.shape(img)[0] * 0.1
        # img = img[gazepoint[1]-y_crop:gazepoint[1]+y_crop, gazepoint[0]-x_crop:gazepoint[0]+x_crop]

        # Match descriptors.
        matches = self.bf.match(self.img_des, self.ref_des)

        # Sort them in the order of their distance.
        matches = sorted(matches, key=lambda x: x.distance)

        # Remove not so good matches
        numGoodMatches = int(len(matches) * self.GOOD_MATCH_PERCENT)
        matches = matches[:numGoodMatches]

        # draw keypoints and matches
        match_img = cv2.drawMatches(self.img, self.img_kp, self.ref, self.ref_kp, matches, None, flags=0)
        cv2.imshow("features", match_img)

        # extract the matched keypoints
        src = np.float32([self.img_kp[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
        dst = np.float32([self.ref_kp[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)

        # Transform human gaze to robot view
        H, mask = cv2.findHomography(src, dst, cv2.RANSAC, 5.0)
        src = np.float32([[[gazepoint[0], gazepoint[1]]]])
        dst = cv2.perspectiveTransform(src, H)

        return dst[0][0]
