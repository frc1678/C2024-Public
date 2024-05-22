import ntcore

from configuration.Configuration import Device


class NTCalibrationController:
    _init_complete: bool = False
    _calibration_mode: ntcore.BooleanEntry
    _wants_frames: ntcore.BooleanEntry

    def _init(self, device: Device):
        if not self._init_complete:
            nt_table = ntcore.NetworkTableInstance.getDefault().getTable(
                "/" + device.device_id + "/calibration"
            )
            self._calibration_mode = nt_table.getBooleanTopic("calib_mode").getEntry(
                False
            )
            self._wants_frames = nt_table.getBooleanTopic("wants_frames").getEntry(
                False
            )
            self._calibration_mode.set(False)
            self._wants_frames.set(False)
            self._init_complete = True

    def get_calibration_mode(self, device: Device) -> bool:
        self._init(device)
        calibrating = self._calibration_mode.get()
        if not calibrating:
            self._wants_frames.set(False)
        return calibrating

    def get_wants_frame(self, device: Device) -> bool:
        self._init(device)
        if self._wants_frames.get():
            self._wants_frames.set(False)
            return True
        return False
