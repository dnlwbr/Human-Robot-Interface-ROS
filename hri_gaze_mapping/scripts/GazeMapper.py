import cv2
import numpy as np


class GazeMapper:
    def __init__(self):
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
        self.parameters = cv2.aruco.DetectorParameters_create()
        self.parameters.markerBorderBits = 2
        # self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.ref, self.img = None, None
        self.ref_corners, self.img_corners = None, None
        self.ref_ids, self.img_ids = None, None
        self.ref_dimensions, self.img_dimensions = None, None

    def update(self, ref_img, img):
        self.ref = ref_img
        self.img = img
        self.ref_dimensions = self.ref.shape
        self.img_dimensions = self.img.shape
        self.ref_corners, self.ref_ids, _ = self.detect(self.ref)
        self.img_corners, self.img_ids, _ = self.detect(self.img)
        self.ref_ids = self.ref_ids.flatten()
        self.img_ids = self.img_ids.flatten()

        if np.all(self.ref_ids is not None):
            self.ref_corners = np.asarray(self.ref_corners).reshape(4*len(self.ref_ids), 2)

        if np.all(self.img_ids is not None):
            self.img_corners = np.asarray(self.img_corners).reshape(4 * len(self.img_ids), 2)

        assert len(self.ref_ids) >= 2, 'Reference image must contain at least two markers'

    def detect(self, img):
        if np.shape(img)[2] == 3:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        else:
            gray = img
        return cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

    def map(self, gazepoint):
        def show_circle(img, gp, radius):
            tmp = img.copy()
            cv2.circle(tmp, (gp[0], gp[1]), radius, (0, 255, 0), 5)
            return tmp

        field = show_circle(self.img, gazepoint, 20)

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

            # Previews
            field_preview = cv2.resize(field, None, fx=0.4, fy=0.4)
            cv2.imshow("field", field_preview)
            robot_gaze = show_circle(self.ref, dst[0][0], 30)
            robot_preview = cv2.resize(robot_gaze, None, fx=0.5, fy=0.5)
            cv2.imshow("Robot with human gaze", robot_preview)

            return src[0][0], dst[0][0]
