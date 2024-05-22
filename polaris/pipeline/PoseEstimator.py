from typing import List
import cv2

import numpy as np
from configuration.Configuration import PolarisConfiguration, Intrinsics
from Observations import FiducialObservation2d, CameraPoseObservation3d
from wpimath.geometry import *
from Util import *


class PoseEstimator:
    def __init__(self):
        pass

    def solve_camera_pose(
        self,
        observations: List[FiducialObservation2d],
        configuration: PolarisConfiguration,
        intrinsics: Intrinsics,
    ) -> CameraPoseObservation3d:
        if configuration.environment.tag_map is None:
            print("NO TAG MAP!")
            return None

        if len(observations) == 0:
            return None

        tag_size = configuration.environment.tag_size_m
        all_object_points = []
        all_image_points = []
        tag_ids = []
        tag_poses = []

        for observation in observations:
            tag_pose = None
            for tag in configuration.environment.tag_map["tags"]:
                if tag["ID"] == observation.tag_id:
                    tag_pose = Pose3d(
                        Translation3d(
                            tag["pose"]["translation"]["x"],
                            tag["pose"]["translation"]["y"],
                            tag["pose"]["translation"]["z"],
                        ),
                        Rotation3d(
                            Quaternion(
                                tag["pose"]["rotation"]["quaternion"]["W"],
                                tag["pose"]["rotation"]["quaternion"]["X"],
                                tag["pose"]["rotation"]["quaternion"]["Y"],
                                tag["pose"]["rotation"]["quaternion"]["Z"],
                            )
                        ),
                    )
                    break

            if tag_pose is None:
                continue

            corner_0 = tag_pose + Transform3d(
                Translation3d(0, tag_size / 2.0, -tag_size / 2.0), Rotation3d()
            )
            corner_1 = tag_pose + Transform3d(
                Translation3d(0, -tag_size / 2.0, -tag_size / 2.0), Rotation3d()
            )
            corner_2 = tag_pose + Transform3d(
                Translation3d(0, -tag_size / 2.0, tag_size / 2.0), Rotation3d()
            )
            corner_3 = tag_pose + Transform3d(
                Translation3d(0, tag_size / 2.0, tag_size / 2.0), Rotation3d()
            )

            all_object_points += [
                wpilibTranslationToOpenCv(corner_0.translation()),
                wpilibTranslationToOpenCv(corner_1.translation()),
                wpilibTranslationToOpenCv(corner_2.translation()),
                wpilibTranslationToOpenCv(corner_3.translation()),
            ]

            all_image_points += [
                [observation.corners[0][0][0], observation.corners[0][0][1]],
                [observation.corners[0][1][0], observation.corners[0][1][1]],
                [observation.corners[0][2][0], observation.corners[0][2][1]],
                [observation.corners[0][3][0], observation.corners[0][3][1]],
            ]

            tag_ids.append(observation.tag_id)
            tag_poses.append(tag_pose)

        # Only one tag seen sad
        if len(tag_ids) == 1:
            object_points = np.array(
                [
                    [-tag_size / 2.0, tag_size / 2.0, 0.0],
                    [tag_size / 2.0, tag_size / 2.0, 0.0],
                    [tag_size / 2.0, -tag_size / 2.0, 0.0],
                    [-tag_size / 2.0, -tag_size / 2.0, 0.0],
                ]
            )
            try:
                _, rvecs, tvecs, errors = cv2.solvePnPGeneric(
                    object_points,
                    np.array(all_image_points),
                    intrinsics.camera_matrix,
                    intrinsics.distortion_coefficients,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE,
                )
            except:
                print("Failed to solve PnP")
                return None

            field_to_tag = tag_poses[0]

            camera_to_tag = openCvPoseToWpilib(tvecs[0], rvecs[0])
            camera_to_tag_transform = Transform3d(
                camera_to_tag.translation(), camera_to_tag.rotation()
            )
            field_to_camera = field_to_tag.transformBy(
                camera_to_tag_transform.inverse()
            )

            camera_to_tag_alt = openCvPoseToWpilib(tvecs[1], rvecs[1])
            camera_to_tag_transform_alt = Transform3d(
                camera_to_tag_alt.translation(), camera_to_tag_alt.rotation()
            )
            field_to_camera_alt = field_to_tag.transformBy(
                camera_to_tag_transform_alt.inverse()
            )

            return CameraPoseObservation3d(
                tag_ids,
                field_to_camera,
                errors[0][0],
                field_to_camera_alt,
                errors[1][0],
            )
        else:
            # Run SolvePNP with all tags
            try:
                _, rvecs, tvecs, errors = cv2.solvePnPGeneric(
                    numpy.array(all_object_points),
                    numpy.array(all_image_points),
                    intrinsics.camera_matrix,
                    intrinsics.distortion_coefficients,
                    flags=cv2.SOLVEPNP_SQPNP,
                )
            except:
                return None

            # Calculate WPILib camera pose
            camera_to_field_pose = openCvPoseToWpilib(tvecs[0], rvecs[0])
            camera_to_field = Transform3d(
                camera_to_field_pose.translation(), camera_to_field_pose.rotation()
            )
            field_to_camera = camera_to_field.inverse()
            field_to_camera_pose = Pose3d(
                field_to_camera.translation(), field_to_camera.rotation()
            )

            # Return result
            return CameraPoseObservation3d(
                tag_ids, field_to_camera_pose, errors[0][0], None, None
            )
