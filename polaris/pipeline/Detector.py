from typing import List, Tuple
from cv2 import aruco, Mat

from Observations import FiducialObservation2d


class Detector:
    _aruco_detector = None
    _tag_dictionary = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36H11)
    _detector_params = aruco.DetectorParameters()

    def __init__(self) -> None:
        self._detector_params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        self._aruco_detector = aruco.ArucoDetector(
            self._tag_dictionary, self._detector_params
        )

    def detectAruco(self, image: Mat) -> List[FiducialObservation2d]:
        corners, ids, rejected = self._aruco_detector.detectMarkers(image)
        if len(corners) == 0:
            return []
        return [
            FiducialObservation2d(id[0], corner) for id, corner in zip(ids, corners)
        ]
