from dataclasses import dataclass
from typing import List

import numpy
from wpimath.geometry import *


@dataclass(frozen=True)
class FiducialObservation2d:
    tag_id: int
    corners: numpy._typing.NDArray[numpy.float64]


@dataclass(frozen=True)
class CameraPoseObservation3d:
    tag_ids: List[int]
    pose: Pose3d
    error: float
    pose_alt: Pose3d
    error_alt: float
