import cv2
import numpy as np


GOOD_MATCH_PERCENT = 0.15


class GazeMapper:
    def __init__(self, ref_filename, videoCap, path):
        self.saveToPath = path
        self.ref = cv2.imread(ref_filename, cv2.IMREAD_ANYCOLOR)
        self.kp, self.des = self.detect(self.ref)
        self.refDimensions = self.ref.shape

        # self.fourcc = cv2.VideoWriter_fourcc(*'mp4v')

        # self.out_field = cv2.VideoWriter(self.saveToPath+'/gaze_out.mp4', self.fourcc, 30,
        #                                  (int(videoCap.get(cv2.CAP_PROP_FRAME_WIDTH)),
        #                                   int(videoCap.get(cv2.CAP_PROP_FRAME_HEIGHT))))
        # self.out_ref = cv2.VideoWriter(self.saveToPath+'/gaze_out_ref.mp4', self.fourcc, 30,
        #                                (int(self.refDimensions[1]), int(self.refDimensions[0])))
        # self.out_ref_nogaze = cv2.VideoWriter(self.saveToPath+'/no_gaze_out_ref.mp4', self.fourcc, 30,
        #                                       (int(self.refDimensions[1]), int(self.refDimensions[0])))

    def detect(self, img):
        if np.shape(img)[2] == 3:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        else:
            gray = img

        # Initiate ORB detector
        orb = cv2.ORB_create()
        # find the keypoints and descriptors with ORB
        kp, des = orb.detectAndCompute(gray, None)
        return kp, des

    def release(self):
        # self.out_field.release()
        # self.out_ref.release()
        # self.out_ref_nogaze.release()
        pass

    def map(self, img, gazepoint):
        def show_circle(img, gp, radius):
            tmp = img.copy()
            cv2.circle(tmp, (gp[0], gp[1]), radius, (0, 255, 0), 5)
            return tmp

        field = show_circle(img, gazepoint, 20)
        # preview = cv2.resize(field, None, fx=0.4, fy=0.4)
        # cv2.imshow("field", preview)
        # self.out_field.write(field)
        cv2.imwrite(self.saveToPath + '/gaze_out.jpg', field)

        # x_crop = 50#np.shape(img)[1] * 0.2
        # y_crop = 50#np.shape(img)[0] * 0.1
        # img = img[gazepoint[1]-y_crop:gazepoint[1]+y_crop, gazepoint[0]-x_crop:gazepoint[0]+x_crop]

        kp, des = self.detect(img)

        # create BFMatcher object
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        # Match descriptors.
        matches = bf.match(des, self.des)

        # Sort them in the order of their distance.
        matches = sorted(matches, key=lambda x: x.distance)

        # Remove not so good matches
        numGoodMatches = int(len(matches) * GOOD_MATCH_PERCENT)
        matches = matches[:numGoodMatches]

        # draw keypoints and matches
        # features_img = cv2.drawKeypoints(img, kp, None, color=(0, 255, 0), flags=0)
        match_img = cv2.drawMatches(img, kp, self.ref, self.kp, matches, None, flags=0)
        cv2.imwrite(self.saveToPath + '/features.jpg', match_img)

        # extract the matched keypoints
        src = np.float32([kp[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
        dst = np.float32([self.kp[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)

        H, mask = cv2.findHomography(src, dst, cv2.RANSAC, 5.0)

        warped_nogaze = cv2.warpPerspective(img, H, (self.ref.shape[1], self.ref.shape[0]))
        # self.out_ref_nogaze.write(warped_nogaze)
        cv2.imwrite(self.saveToPath + '/no_gaze_out_ref.jpg', warped_nogaze)

        robot_img = show_circle(img, gazepoint, 20)
        warped_gaze = cv2.warpPerspective(robot_img, H, (self.ref.shape[1], self.ref.shape[0]))
        # warped_preview = cv2.resize(warped_gaze, None, fx=0.1, fy=0.1)
        # cv2.imshow("ref", warped_preview)
        # self.out_ref.write(warped_gaze)
        cv2.imwrite(self.saveToPath + '/gaze_out_ref.jpg', warped_gaze)

        # in robot_gaze coordinate
        src = np.float32([[[gazepoint[0], gazepoint[1]]]])
        dst = cv2.perspectiveTransform(src, H)
        cv2.imwrite(self.saveToPath + '/robot_gaze.jpg', show_circle(self.ref, dst[0][0], 50))
