import time
import cv2
import numpy as np
import ntcore

from configuration.Configuration import PolarisConfiguration
from configuration.ConfigurationRetriever import ConfigurationRetriever
from calibration.Calibration import Calibrator
from calibration.CalibrationController import NTCalibrationController
from Observations import FiducialObservation2d
from output.StreamServer import MjpegServer
from output.NTPublisher import NTPublisher

from pipeline.PoseEstimator import PoseEstimator
from pipeline.Detector import Detector
from pipeline.Capture import GStreamerCapture

calibration_started: bool = False

if __name__ == "__main__":
    print("Using OpenCV Version %s" % (cv2.__version__))

    configuration = PolarisConfiguration()

    capture = GStreamerCapture()
    processor = Detector()
    pose_estimator = PoseEstimator()
    calibrator = Calibrator()
    sub = ConfigurationRetriever()
    pub = NTPublisher()
    calibration_controller = NTCalibrationController()
    stream = MjpegServer()

    sub.updateLocal(configuration)

    stream.start(configuration.device)

    frame_count = 0
    last_print = 0

    retval, frame = capture.get_frame(configuration.camera)

    ntcore.NetworkTableInstance.getDefault().setServer("10.16.78.2")
    ntcore.NetworkTableInstance.getDefault().startClient4(
        configuration.device.device_id
    )

    while configuration.environment.tag_map is None:
        sub.updateNT(configuration)

    while True:
        retval, frame = capture.get_frame(configuration.camera)
        timestamp = time.time()

        if not retval:
            time.sleep(0.5)
            continue

        fps = None
        frame_count += 1
        if time.time() - last_print > 1:
            last_print = time.time()
            fps = frame_count
            print("Running at", frame_count, "fps")
            frame_count = 0

        if calibration_controller.get_calibration_mode(configuration.device):
            calibration_started = True
            calibrator.process_frame(
                frame, calibration_controller.get_wants_frame(configuration.device)
            )

        elif calibration_started:
            calibrator.finish()
            time.sleep(5)
            sub.updateLocal(configuration)
            calibration_started = False

        else:
            # Detect 2d markers and draw on frame
            markers = processor.detectAruco(frame)
            cv2.aruco.drawDetectedMarkers(
                frame,
                [m.corners for m in markers],
                np.fromiter((m.tag_id for m in markers), dtype=int),
            )
            observations2d = [
                FiducialObservation2d(m.tag_id, m.corners) for m in markers
            ]

            # Pass in markers to pose estimator
            pose_observation = pose_estimator.solve_camera_pose(
                observations2d, configuration, configuration.intrinsics
            )

            # Lick the stamp and send it
            pub.send(configuration.device, timestamp, pose_observation, fps)

        stream.set_frame(frame)
