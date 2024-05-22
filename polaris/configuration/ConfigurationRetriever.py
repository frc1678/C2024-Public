import json
import cv2
import ntcore
import math

import numpy
from configuration.Configuration import Intrinsics, PolarisConfiguration


class ConfigurationRetriever:
    CONFIG_FILENAME = "Config.json"
    CALIBRATION_FILENAME = "Calibration.yaml"

    _nt_initialized: bool = False
    _camera_id_sub: ntcore.IntegerSubscriber
    _camera_resolution_width_sub: ntcore.IntegerSubscriber
    _camera_resolution_height_sub: ntcore.IntegerSubscriber
    _camera_auto_exposure_sub: ntcore.IntegerSubscriber
    _camera_exposure_sub: ntcore.IntegerSubscriber
    _camera_gain_sub: ntcore.IntegerSubscriber
    _fiducial_size_m_sub: ntcore.DoubleSubscriber
    _tag_layout_sub: ntcore.DoubleSubscriber

    def updateLocal(self, configuration: PolarisConfiguration):
        with open(self.CONFIG_FILENAME, "r") as config_file:
            config_data = json.loads(config_file.read())
            configuration.device.device_id = config_data["device_id"]
            configuration.device.server_ip = config_data["server_ip"]
            configuration.device.stream_port = config_data["stream_port"]

        calib_data = cv2.FileStorage(self.CALIBRATION_FILENAME, cv2.FileStorage_READ)
        camera_matrix = calib_data.getNode("camera_matrix").mat()
        distort_coeffs = calib_data.getNode("distortion_coefficients").mat()
        if (
            type(camera_matrix) == numpy.ndarray
            and type(distort_coeffs) == numpy.ndarray
        ):
            print("saved calibration data")
            configuration.intrinsics.camera_matrix = camera_matrix
            configuration.intrinsics.distortion_coefficients = distort_coeffs

    def updateNT(self, configuration: PolarisConfiguration) -> None:
        if not self._nt_initialized:
            nt_table = ntcore.NetworkTableInstance.getDefault().getTable(
                "/" + configuration.device.device_id + "/configs"
            )
            self._camera_id_sub = nt_table.getDoubleTopic("camera_id").subscribe(
                configuration.camera.id
            )
            self._camera_resolution_width_sub = nt_table.getDoubleTopic(
                "camera_resolution_width"
            ).subscribe(configuration.camera.res_w)
            self._camera_resolution_height_sub = nt_table.getDoubleTopic(
                "camera_resolution_height"
            ).subscribe(configuration.camera.res_h)
            self._camera_auto_exposure_sub = nt_table.getDoubleTopic(
                "camera_auto_exposure"
            ).subscribe(configuration.camera.auto_exposure)
            self._camera_exposure_sub = nt_table.getDoubleTopic(
                "camera_exposure"
            ).subscribe(configuration.camera.exposure)
            self._camera_gain_sub = nt_table.getDoubleTopic("camera_gain").subscribe(
                configuration.camera.gain
            )
            self._fiducial_size_m_sub = nt_table.getDoubleTopic(
                "fiducial_size_m"
            ).subscribe(configuration.environment.tag_size_m)
            self._tag_layout_sub = nt_table.getStringTopic("tag_layout").subscribe("")
            self._init_complete = True

        configuration.camera.id = math.floor(self._camera_id_sub.get())
        configuration.camera.res_w = math.floor(self._camera_resolution_width_sub.get())
        configuration.camera.res_h = math.floor(
            self._camera_resolution_height_sub.get()
        )
        configuration.camera.auto_exposure = math.floor(
            self._camera_auto_exposure_sub.get()
        )
        configuration.camera.exposure = math.floor(self._camera_exposure_sub.get())
        configuration.camera.gain = math.floor(self._camera_gain_sub.get())
        configuration.environment.tag_size_m = self._fiducial_size_m_sub.get()

        try:
            configuration.environment.tag_map = json.loads(self._tag_layout_sub.get())
        except:
            print("Tag map not present, defaulting")
