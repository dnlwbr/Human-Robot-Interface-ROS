import cv2
import numpy as np


class GazeMapper:
    def __init__(self, ref_file, path):
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
        self.parameters = cv2.aruco.DetectorParameters_create()
        self.parameters.markerBorderBits = 2
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.saveToPath = path

        self.ref = ref_file
        self.corners, self.ids, _ = self.detect(self.ref)

        self.ids = self.ids.flatten()
        self.refDimensions = self.ref.shape
        self.corners = np.asarray(self.corners).reshape(4*len(self.ids), 2)

        assert len(self.ids) >= 4, 'Reference image must contain at least four markers'

    def detect(self, img):
        if np.shape(img)[2] == 3:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        else:
            gray = img
        return cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

    def map(self, img, gazepoint):
        def show_circle(img, gp, radius):
            tmp = img.copy()
            cv2.circle(tmp, (gp[0], gp[1]), radius, (0, 255, 0), 5)
            return tmp

        field = show_circle(img, gazepoint, 20)
        # field_preview = cv2.resize(field, None, fx=0.4, fy=0.4)
        # cv2.imshow("field", field_preview)
        cv2.imwrite(self.saveToPath + '/field.jpg', field)

        corners, ids, _ = self.detect(img)

        if np.all(ids is not None):
            ids = ids.flatten()
            corners = np.asarray(corners).reshape(4*len(ids), 2)
            matching_ids = np.intersect1d(self.ids, ids)
            if len(matching_ids) > 0:
                src = []
                dst = []

                def addCorners(matching_id, ids_list, corners_list, matched_corners_list):
                    idx = np.nonzero(matching_id == ids_list)[0][0]
                    for corner in corners_list[4*idx:4*idx+4, :]:
                        matched_corners_list.append(corner)

                for matching_id in matching_ids:
                    addCorners(matching_id, ids, corners, src)  # from current frame
                    addCorners(matching_id, self.ids, self.corners, dst)  # to reference

                assert len(src) == len(dst)

                src = np.float32(src)
                dst = np.float32(dst)

                # Transform human gaze to robot view
                H, mask = cv2.findHomography(src, dst)
                src = np.float32([[[gazepoint[0], gazepoint[1]]]])
                dst = cv2.perspectiveTransform(src, H)

                # Robot
                robot_gaze = show_circle(self.ref, dst[0][0], 30)
                cv2.imwrite(self.saveToPath + '/robot_gaze.jpg', robot_gaze)
                robot_preview = cv2.resize(robot_gaze, None, fx=0.5, fy=0.5)
                cv2.imshow("Robot with human gaze", robot_preview)

                return src[0][0], dst[0][0]
