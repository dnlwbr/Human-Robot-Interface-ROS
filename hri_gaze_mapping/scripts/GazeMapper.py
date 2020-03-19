import cv2
import numpy as np


class GazeMapper:

    def __init__(self):
        self.img, self.ref = None, None
        self.img_dimensions, self.ref_dimensions = None, None
        self.human_gp = None

    def update(self, img, ref_img, human_gp=None):
        self.img = img
        self.ref = ref_img
        self.img_dimensions = self.img.shape
        self.ref_dimensions = self.ref.shape
        self.human_gp = human_gp


class GazeMapperAruco(GazeMapper):

    def __init__(self):
        GazeMapper.__init__(self)
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
        self.parameters = cv2.aruco.DetectorParameters_create()
        self.parameters.markerBorderBits = 2
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.img_ids, self.img_corners = None, None
        self.ref_ids, self.ref_corners = None, None

    def update(self, img, ref_img, human_gp=None):
        GazeMapper.update(self, img, ref_img, human_gp)
        self.img_corners, self.img_ids, _ = self.detect(self.img)
        self.ref_corners, self.ref_ids, _ = self.detect(self.ref)

        if self.img_ids is not None:
            self.img_ids = self.img_ids.flatten()
            self.img_corners = np.asarray(self.img_corners).reshape(4 * len(self.img_ids), 2)

        if self.ref_ids is not None:
            self.ref_ids = self.ref_ids.flatten()
            self.ref_corners = np.asarray(self.ref_corners).reshape(4 * len(self.ref_ids), 2)

        if self.img_ids is not None and self.ref_ids is not None and len(
                np.intersect1d(self.img_ids, self.ref_ids)) > 1:
            return True
        else:
            return False

    def detect(self, img):
        if np.shape(img)[2] == 3:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        else:
            gray = img
        return cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

    def map(self, gazepoint):
        assert self.img_ids is not None, 'Field image must contain at least one marker'
        assert self.ref_ids is not None, 'Reference image must contain at least one marker'

        matching_ids = np.intersect1d(self.ref_ids, self.img_ids)
        if len(matching_ids) > 0:
            src = []
            dst = []

            def addCorners(matching_id, ids_list, corners_list, matched_corners_list):
                idx = np.nonzero(matching_id == ids_list)[0][0]
                for corner in corners_list[4*idx:4*idx+4, :]:
                    matched_corners_list.append(corner)

            for matching_id in matching_ids:
                addCorners(matching_id, self.img_ids, self.img_corners, src)  # from current frame
                addCorners(matching_id, self.ref_ids, self.ref_corners, dst)  # to reference

            assert len(src) == len(dst)

            src = np.float32(src)
            dst = np.float32(dst)

            # Transform human gaze to robot view
            H, mask = cv2.findHomography(src, dst)
            src = np.float32([[[gazepoint[0], gazepoint[1]]]])
            dst = cv2.perspectiveTransform(src, H)

            return dst[0][0]

    def print_warning(self, field_preview, robot_preview, font, font_scale, thickness, margin, text_height, line_height):
        if self.img_ids is None or self.ref_ids is None:
            warning = "No markers detected"
            warning_width = (cv2.getTextSize(warning, font, font_scale, thickness))[0][0]
            color = (0, 0, 255)
            if self.img_ids is None:
                x = field_preview.shape[1] - margin - warning_width
                y = margin + text_height + 0 * line_height
                cv2.putText(field_preview, warning, (x, y), font, font_scale, color, thickness)
            if self.ref_ids is None:
                x = robot_preview.shape[1] - margin - warning_width
                y = margin + text_height + 0 * line_height
                cv2.putText(robot_preview, warning, (x, y), font, font_scale, color, thickness)


class GazeMapperFeature(GazeMapper):

    def __init__(self, descriptors="orb"):
        GazeMapper.__init__(self)
        self.img_kp, self.img_des = None, None
        self.ref_kp, self.ref_des = None, None
        self.matches = None
        self.descriptors = descriptors
        if descriptors == "orb":
            self.orb = cv2.ORB_create()  # Initiate ORB detector
            self.GOOD_MATCH_PERCENT = 0.1
            self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)  # create BFMatcher object
        elif descriptors == "sift":
            self.sift = cv2.xfeatures2d.SIFT_create()  # Initiate SIFT detector
            self.FLANN_INDEX_KDTREE = 1
            self.index_params = dict(algorithm=self.FLANN_INDEX_KDTREE, trees=5)
            self.search_params = dict(checks=50)  # or pass empty dictionary
            self.matcher = cv2.FlannBasedMatcher(self.index_params, self.search_params)
            # self.matcher = cv2.BFMatcher()
        else:
            raise ValueError("Invalid descriptor")

    def crop(self):
        if self.human_gp is not None:
            xy_crop = min(np.shape(self.img)[0], np.shape(self.img)[1]) * 0.2
            x_min = int(max(0, self.human_gp[0]-xy_crop))
            x_max = int(min(np.shape(self.img)[1]-1, self.human_gp[0]+xy_crop))
            y_min = int(max(0, self.human_gp[1]-xy_crop))
            y_max = int(min(np.shape(self.img)[0]-1, self.human_gp[1]+xy_crop))
            self.img = self.img[y_min:y_max, x_min:x_max]

    def update(self, img, ref_img, human_gp=None):
        GazeMapper.update(self, img, ref_img, human_gp)
        self.ref = cv2.resize(self.ref, (self.img_dimensions[1], self.img_dimensions[0]))
        self.crop()  # Crop human image around gaze point
        self.img_kp, self.img_des = self.detect(self.img)
        self.ref_kp, self.ref_des = self.detect(self.ref)

        if None not in [self.img_kp, self.ref_kp]:
            self.matches = self.match(self.img_des, self.ref_des)
            if len(self.matches) >= 4:
                return True
            else:
                return False
        else:
            return False

    def detect(self, img):
        if self.descriptors == "orb":
            # return the keypoints and descriptors found with ORB
            return self.orb.detectAndCompute(img, None)
        elif self.descriptors == "sift":
            # return the keypoints and descriptors found with SIFT
            return self.sift.detectAndCompute(img, None)
        else:
            raise ValueError("Invalid descriptor")

    def match(self, des1, des2):
        if self.descriptors == "orb":
            # Match descriptors
            matches = self.matcher.match(des1, des2)
            # Sort them in the order of their distance.
            matches = sorted(matches, key=lambda x: x.distance)
            # Remove not so good matches
            numGoodMatches = int(len(matches) * self.GOOD_MATCH_PERCENT)
            matches = matches[:numGoodMatches]
        elif self.descriptors == "sift":
            # Match descriptors
            matches = self.matcher.knnMatch(des1, des2, k=2)
            # Apply ratio test
            good = []
            # distances = []
            for m, n in matches:
                if m.distance < 0.7 * n.distance:
                    good.append(m)
                    # distances.append(m.distance)
            matches = good
            # matches = [x for _, x in sorted(zip(distances, good))]
            # matches = matches[0:5]
        else:
            raise ValueError("Invalid descriptor")

        # draw keypoints and matches
        match_img = cv2.drawMatches(self.img, self.img_kp, self.ref, self.ref_kp, matches,
                                    None)  # , flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
        cv2.imshow("features", match_img)
        return matches

    def map(self, gazepoint):

        # extract the matched keypoints
        src = np.float32([self.img_kp[m.queryIdx].pt for m in self.matches]).reshape(-1, 1, 2)
        dst = np.float32([self.ref_kp[m.trainIdx].pt for m in self.matches]).reshape(-1, 1, 2)

        # Transform human gaze to robot view
        H, mask = cv2.findHomography(src, dst, cv2.RANSAC, 5.0)
        src = np.float32([[[gazepoint[0], gazepoint[1]]]])
        dst = cv2.perspectiveTransform(src, H)

        return dst[0][0]
