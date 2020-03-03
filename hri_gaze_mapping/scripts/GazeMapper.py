import cv2
import numpy as np


class GazeMapper:
    def __init__(self):
        self.ref, self.img = None, None
        self.ref_dimensions, self.img_dimensions = None, None
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
        self.parameters = cv2.aruco.DetectorParameters_create()
        self.parameters.markerBorderBits = 2
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.ref_corners, self.img_corners = None, None
        self.ref_ids, self.img_ids = None, None

    def update(self, img, ref_img):
        self.img = img
        self.ref = ref_img
        self.img_dimensions = self.img.shape
        self.ref_dimensions = self.ref.shape
        self.img_corners, self.img_ids, _ = self.detect(self.img)
        self.ref_corners, self.ref_ids, _ = self.detect(self.ref)

        if self.img_ids is not None:
            self.img_ids = self.img_ids.flatten()
            self.img_corners = np.asarray(self.img_corners).reshape(4 * len(self.img_ids), 2)

        if self.ref_ids is not None:
            self.ref_ids = self.ref_ids.flatten()
            self.ref_corners = np.asarray(self.ref_corners).reshape(4*len(self.ref_ids), 2)

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
