import math
from typing import List
import ntcore

from configuration.Configuration import Device
from Observations import CameraPoseObservation3d


class NTPublisher:
    _initialized: bool = False
    _observations_publisher: ntcore.DoubleArrayPublisher
    _raw_pose_publisher: ntcore.DoubleArrayPublisher
    _fps_publisher: ntcore.IntegerPublisher

    def send(
        self,
        config: Device,
        timestamp: float,
        observation: CameraPoseObservation3d,
        fps: int,
    ):
        if not self._initialized:
            table = ntcore.NetworkTableInstance.getDefault().getTable(
                "/%s/output" % config.device_id
            )
            self._observations_publisher = table.getDoubleArrayTopic(
                "observations"
            ).publish(
                ntcore.PubSubOptions(periodic=0, sendAll=True, keepDuplicates=True)
            )
            self._raw_pose_publisher = table.getDoubleArrayTopic("raw_poses").publish(
                ntcore.PubSubOptions(periodic=0, sendAll=True, keepDuplicates=True)
            )
            self._fps_publisher = table.getIntegerTopic("fps").publish()

        if fps != None:
            self._fps_publisher.set(fps)

        observation_data: List[float] = [0]
        raw_poses: List[float] = [0]
        if observation != None:
            observation_data[0] = 1
            observation_data.append(observation.error)
            observation_data.append(observation.pose.translation().X())
            observation_data.append(observation.pose.translation().Y())
            observation_data.append(observation.pose.translation().Z())
            observation_data.append(observation.pose.rotation().getQuaternion().W())
            observation_data.append(observation.pose.rotation().getQuaternion().X())
            observation_data.append(observation.pose.rotation().getQuaternion().Y())
            observation_data.append(observation.pose.rotation().getQuaternion().Z())

            raw_poses[0] = observation.pose.translation().X()
            raw_poses.append(observation.pose.translation().Y())
            raw_poses.append(observation.pose.translation().Z())
            raw_poses.append(observation.pose.rotation().getQuaternion().W())
            raw_poses.append(observation.pose.rotation().getQuaternion().X())
            raw_poses.append(observation.pose.rotation().getQuaternion().Y())
            raw_poses.append(observation.pose.rotation().getQuaternion().Z())

            if observation.error_alt != None and observation.pose_alt != None:
                observation_data[0] = 2
                observation_data.append(observation.error_alt)
                observation_data.append(observation.pose_alt.translation().X())
                observation_data.append(observation.pose_alt.translation().Y())
                observation_data.append(observation.pose_alt.translation().Z())
                observation_data.append(
                    observation.pose_alt.rotation().getQuaternion().W()
                )
                observation_data.append(
                    observation.pose_alt.rotation().getQuaternion().X()
                )
                observation_data.append(
                    observation.pose_alt.rotation().getQuaternion().Y()
                )
                observation_data.append(
                    observation.pose_alt.rotation().getQuaternion().Z()
                )

                raw_poses.append(observation.pose_alt.translation().X())
                raw_poses.append(observation.pose_alt.translation().Y())
                raw_poses.append(observation.pose_alt.translation().Z())
                raw_poses.append(observation.pose_alt.rotation().getQuaternion().W())
                raw_poses.append(observation.pose_alt.rotation().getQuaternion().X())
                raw_poses.append(observation.pose_alt.rotation().getQuaternion().Y())
                raw_poses.append(observation.pose_alt.rotation().getQuaternion().Z())

            for tag_id in observation.tag_ids:
                observation_data.append(tag_id)

        self._observations_publisher.set(
            observation_data, math.floor(timestamp * 1000000)
        )

    # self._raw_pose_publisher.set(
    # raw_poses, math.floor(timestamp * 1000000))
