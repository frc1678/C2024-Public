from dataclasses import dataclass
import numpy


@dataclass
class Device:
    device_id: str = "polaris"
    server_ip: str = "10.16.78.2"
    stream_port: int = 8000


@dataclass
class Camera:
    id: int = 0
    res_w: int = 1600
    res_h: int = 1200
    gain: int = 25
    fps: int = 50
    auto_exposure: int = 1
    exposure: int = 1


@dataclass
class Environment:
    tag_size_m: float = 0.1651
    tag_map: any = None


@dataclass
class Intrinsics:
    camera_matrix: numpy._typing.NDArray[numpy.float64] = numpy.array([])
    distortion_coefficients: numpy._typing.NDArray[numpy.float64] = numpy.array([])


# Main config class
@dataclass
class PolarisConfiguration:
    camera: Camera = Camera()
    intrinsics: Intrinsics = Intrinsics()
    device: Device = Device()
    environment: Environment = Environment()
