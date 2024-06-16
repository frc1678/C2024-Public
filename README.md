# C2024-Public
Code for Team 1678's 2024 robot, Nik.

See also:
- Our CAD Release and Scouting Whitepaper
- Code from [2023](https://github.com/frc1678/C2023-Public/), and [2022](https://github.com/frc1678/C2022-Public/).

![Robot Image](/images/Champs.jpg)

## Highlights
- Drivetrain

    The swerve drivetrain features a [custom Phoenix 6 enabled odometry class](/src/main/java/com/team1678/frc2024/subsystems/WheelTracker.java) and [pure pursuit path following](/src/main/java/com/team1678/lib/swerve/DriveMotionPlanner.java) for consistent autonomous driving.

- Vision Pose Correction

    The robot continously records and stores the most recent vision and odometry updates in a [robot state class](/src/main/java/com/team1678/frc2024/RobotState.java#L33), and feeds the information into an [extended kalman filter](/src/main/java/com/team1678/frc2024/RobotState.java#L114). The localization factors in [number of tags seen, average distance to tags, and closest tag](/src/main/java/com/team1678/frc2024/subsystems/vision/VisionDevice.java#L69) when calculating trust factor.

- Auto Aim

    Using the relative transform to either the speaker or ferry target, swerve drivetrain overrides driver and path heading control when instructed to auto-align. The [superstructure](/src/main/java/com/team1678/frc2024/subsystems/Superstructure.java) uses the pose from localization and [multiple regression maps](/src/main/java/com/team1678/frc2024/shooting/RegressionMaps.java) to calculate the target drivetrain angle, shooter angle, and shooter RPM for either [shooting](/src/main/java/com/team1678/frc2024/shooting/ShootingUtil.java) or [ferrying](/src/main/java/com/team1678/frc2024/shooting/FerryUtil.java).

- Superstructure Control

    The elevator, intake, three indexer rollers are controlled using a [request queue](/src/main/java/com/team1678/lib/requests/). Desired actions are added to the queue by calling the appropiate [state or transition method](/src/main/java/com/team1678/frc2024/subsystems/Superstructure.java#L367), which adds a chain of requests to be sequentially run.

## Notable Package Functions
- [`com.team1678.frc2024.auto`](/src/main/java/com/team1678/frc2024/auto/)

    Contains structure code for defining and running autonomous modes. 

- [`com.team1678.frc2024.auto.actions`](/src/main/java/com/team1678/frc2024/auto/actions)

    Contains all actions used during the autonomous, which all inherit the [`Action`](/src/main/java/com/team1678/frc2024/auto/actions/Action.java) interface. Examples include  [driving paths](/src/main/java/com/team1678/frc2024/auto/actions/Action.java), [waiting for a superstructure command](/src/main/java/com/team1678/frc2024/auto/actions/WaitForSuperstructureAction.java), and [waiting for a target track within error](/src/main/java/com/team1678/frc2024/auto/actions/WaitForTargetTrackAction.java). 
- [`com.team1678.frc2024.auto.modes`](/src/main/java/com/team1678/frc2024/auto/modes/)

    Contains all autonomous modes. Notable modes are the [customizable six note modes](/src/main/java/com/team1678/frc2024/auto/modes/adaptive/AdaptiveSixMode.java), the [note detection three note modes](/src/main/java/com/team1678/frc2024/auto/modes/three/), and the [mandatory do nothing mode](/src/main/java/com/team1678/frc2024/auto/modes/DoNothingMode.java). 

- [`com.team1678.frc2024.controlboard`](/src/main/java/com/team1678/frc2024/controlboard/)

    Contains code for handling [driver input](/src/main/java/com/team1678/frc2024/controlboard/DriverControls.java). Nik is controlled by two [Xbox Controllers](/src/main/java/com/team1678/frc2024/controlboard/CustomXboxController.java).

- [`com.team1678.frc2024.loops`](/src/main/java/com/team1678/frc2024/loops/)

    Contains code for loops, which run sections of code periodically on the robot. All loops implement the [`Loop`](/src/main/java/com/team1678/frc2024/loops/Loop.java) interface, and are handled by the [`Looper`](/src/main/java/com/team1678/frc2024/loops/Looper.java) class. We run our loops at 50 Hz, or every 0.02 seconds.

- [`com.team1678.frc2024.paths`](/src/main/java/com/team1678/frc2024/paths)

    Contains the [`TrajectoryGenerator`](/src/main/java/com/team1678/frc2024/paths/TrajectoryGenerator.java) class which contains the trajectories that the robot drives during autonomous mode. Each Trajectory is composed of a list of Pose2d objects and headings.

- [`com.team1678.frc2024.planners`](/src/main/java/com/team1678/frc2024/planners)

    Contains the [`ElevatorMotionPlanner`](/src/main/java/com/team1678/frc2024/planners/ElevatorMotionPlanner.java) class that contains request chains for extending and retracting the elevator to ensure it does not collide with the intake.

- [`com.team1678.frc2024.shooting`](/src/main/java/com/team1678/frc2024/shooting/)

    Contains util classes related to solving for shooting parameters for [ferrying](/src/main/java/com/team1678/frc2024/shooting/FerryUtil.java) and [speaker shooting](/src/main/java/com/team1678/frc2024/shooting/ShootingUtil.java), and [tuned regression maps](/src/main/java/com/team1678/frc2024/shooting/RegressionMaps.java) referenced to find hood angle, shooter rpm, and yaw offset.

- [`com.team1678.frc2024.subsystems`](/src/main/java/com/team1678/frc2024/subsystems/)

    Contains classes for each subsystem on the robot, all of which extend the [`Subsystem`](/src/main/java/com/team1678/frc2024/subsystems/Subsystem.java) abstract class. Each subsystem uses state machines for control. Subsystems also contain an enabled loop, a read periodic inputs method, and a write periodic outputs method, which are controlled by the [SubystemManager](/src/main/java/com/team1678/frc2024/SubsystemManager.java) class.