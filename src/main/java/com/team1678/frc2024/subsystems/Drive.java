package com.team1678.frc2024.subsystems;

import com.team1678.frc2024.Constants;
import com.team1678.frc2024.Constants.SwerveConstants;
import com.team1678.frc2024.Constants.SwerveConstants.Mod0;
import com.team1678.frc2024.Constants.SwerveConstants.Mod1;
import com.team1678.frc2024.Constants.SwerveConstants.Mod2;
import com.team1678.frc2024.Constants.SwerveConstants.Mod3;
import com.team1678.frc2024.RobotState;
import com.team1678.frc2024.loops.ILooper;
import com.team1678.frc2024.loops.Loop;
import com.team1678.lib.Util;
import com.team1678.lib.drivers.Pigeon;
import com.team1678.lib.logger.LogUtil;
import com.team1678.lib.swerve.ChassisSpeeds;
import com.team1678.lib.swerve.DriveMotionPlanner;
import com.team1678.lib.swerve.DriveMotionPlanner.FollowerType;
import com.team1678.lib.swerve.SwerveDriveKinematics;
import com.team1678.lib.swerve.SwerveHeadingController;
import com.team1678.lib.swerve.SwerveModule;
import com.team1678.lib.swerve.SwerveModulePosition;
import com.team1678.lib.swerve.SwerveModuleState;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithMotion;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.trajectory.TimedView;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.TimedState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.List;

public class Drive extends Subsystem {

	public enum DriveControlState {
		FORCE_ORIENT,
		OPEN_LOOP,
		HEADING_CONTROL,
		VELOCITY,
		PATH_FOLLOWING
	}

	private WheelTracker mWheelTracker;
	private Pigeon mPigeon = Pigeon.getInstance();
	public SwerveModule[] mModules;

	private PeriodicIO mPeriodicIO = new PeriodicIO();
	private DriveControlState mControlState = DriveControlState.FORCE_ORIENT;

	private boolean odometryReset = false;

	private final DriveMotionPlanner mMotionPlanner;
	private final SwerveHeadingController mHeadingController;

	private Translation2d enableFieldToOdom = null;

	private boolean mOverrideTrajectory = false;
	private boolean mOverrideHeading = false;

	private Rotation2d mTrackingAngle = Rotation2d.identity();

	private KinematicLimits mKinematicLimits = SwerveConstants.kUncappedLimits;

	private static Drive mInstance;

	public static Drive getInstance() {
		if (mInstance == null) {
			mInstance = new Drive();
		}
		return mInstance;
	}

	private Drive() {
		mModules = new SwerveModule[] {
			new SwerveModule(
					0, Mod0.SwerveModuleConstants(), Cancoders.getInstance().getFrontLeft()),
			new SwerveModule(
					1, Mod1.SwerveModuleConstants(), Cancoders.getInstance().getFrontRight()),
			new SwerveModule(
					2, Mod2.SwerveModuleConstants(), Cancoders.getInstance().getBackLeft()),
			new SwerveModule(
					3, Mod3.SwerveModuleConstants(), Cancoders.getInstance().getBackRight())
		};

		mMotionPlanner = new DriveMotionPlanner();
		mHeadingController = new SwerveHeadingController();

		mPigeon.setYaw(0.0);
		mWheelTracker = new WheelTracker(mModules);
	}

	public void setKinematicLimits(KinematicLimits newLimits) {
		this.mKinematicLimits = newLimits;
	}

	/**
	 * Updates drivetrain with latest desired speeds from the joystick, and sets DriveControlState appropriately.
	 *
	 * @param speeds ChassisSpeeds object derived from joystick input.
	 */
	public void feedTeleopSetpoint(ChassisSpeeds speeds) {
		if (mControlState == DriveControlState.PATH_FOLLOWING) {
			if (Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
					> mKinematicLimits.kMaxDriveVelocity * 0.1) {
				mControlState = DriveControlState.OPEN_LOOP;
			} else {
				return;
			}
		} else if (mControlState == DriveControlState.HEADING_CONTROL) {
			if (Math.abs(speeds.omegaRadiansPerSecond) > 1.0) {
				mControlState = DriveControlState.OPEN_LOOP;
			} else {
				double x = speeds.vxMetersPerSecond;
				double y = speeds.vyMetersPerSecond;
				double omega = mHeadingController.update(mPeriodicIO.heading.getRadians(), Timer.getFPGATimestamp());
				mPeriodicIO.des_chassis_speeds = new ChassisSpeeds(x, y, omega);
				return;
			}
		} else if (mControlState != DriveControlState.OPEN_LOOP) {
			mControlState = DriveControlState.OPEN_LOOP;
		}

		mPeriodicIO.des_chassis_speeds = speeds;
	}

	public void setOpenLoop(ChassisSpeeds speeds) {
		mPeriodicIO.des_chassis_speeds = speeds;
		if (mControlState != DriveControlState.OPEN_LOOP) {
			mControlState = DriveControlState.OPEN_LOOP;
		}
	}

	public void setVelocity(ChassisSpeeds speeds) {
		mPeriodicIO.des_chassis_speeds = speeds;
		if (mControlState != DriveControlState.VELOCITY) {
			mControlState = DriveControlState.VELOCITY;
		}
	}

	/**
	 * Instructs the drivetrain to snap heading to target angle.
	 *
	 * @param angle Rotation2d angle to snap to.
	 */
	public void snapHeading(Rotation2d angle) {
		if (mControlState != DriveControlState.HEADING_CONTROL && mControlState != DriveControlState.PATH_FOLLOWING) {
			mControlState = DriveControlState.HEADING_CONTROL;
		}
		mHeadingController.setSnapTarget(angle.getRadians());
	}

	/**
	 * Instructs the drivetrain to stabilize heading around target angle.
	 *
	 * @param angle Target angle to stabilize around.
	 */
	public void stabilizeHeading(Rotation2d angle) {
		if (mControlState != DriveControlState.HEADING_CONTROL && mControlState != DriveControlState.PATH_FOLLOWING) {
			mControlState = DriveControlState.HEADING_CONTROL;
		}
		if (mHeadingController.getTargetHeadingRadians() != angle.getRadians()) {
			mHeadingController.setStabilizeTarget(angle.getRadians());
		}
	}

	/**
	 * Enable/disables vision heading control.
	 *
	 * @param value Whether or not to override rotation joystick with vision target. 
	 */
	public synchronized void overrideHeading(boolean value) {
		mOverrideHeading = value;
	}

	/**
	 * Updates needed angle to track a goal.
	 *
	 * @param angle Sets the wanted robot heading to track a goal.
	 */
	public synchronized void feedTrackingSetpoint(Rotation2d angle) {
		mTrackingAngle = angle;
	}

	/**
	 * Stops modules in place.
	 */
	public synchronized void stopModules() {
		List<Rotation2d> orientations = new ArrayList<>();
		for (SwerveModuleState SwerveModuleState : mPeriodicIO.des_module_states) {
			orientations.add(SwerveModuleState.angle);
		}
		orientModules(orientations);
	}

	/**
	 * Orients modules to the angles provided.
	 * @param orientations Rotation2d of target angles, indexed by module number.
	 */
	public synchronized void orientModules(List<Rotation2d> orientations) {
		if (mControlState != DriveControlState.FORCE_ORIENT) {
			mControlState = DriveControlState.FORCE_ORIENT;
		}
		for (int i = 0; i < mModules.length; ++i) {
			mPeriodicIO.des_module_states[i] = new SwerveModuleState(0.0, orientations.get(i));
		}
	}

	@Override
	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {
				mPeriodicIO.des_chassis_speeds = new ChassisSpeeds();
				mControlState = DriveControlState.OPEN_LOOP;
				mWheelTracker.start();
			}

			@Override
			public void onLoop(double timestamp) {
				synchronized (Drive.this) {
					switch (mControlState) {
						case PATH_FOLLOWING:
							updatePathFollower();
							break;
						case HEADING_CONTROL:
							break;
						case OPEN_LOOP:
						case VELOCITY:
						case FORCE_ORIENT:
							break;
						default:
							stop();
							break;
					}
					updateSetpoint();

					RobotState.getInstance()
							.addOdometryUpdate(
									timestamp,
									mWheelTracker.getRobotPose(),
									mPeriodicIO.measured_velocity,
									mPeriodicIO.predicted_velocity);
				}
			}

			@Override
			public void onStop(double timestamp) {
				mPeriodicIO.des_chassis_speeds = new ChassisSpeeds();
				mControlState = DriveControlState.OPEN_LOOP;
				mWheelTracker.stop();
				enableFieldToOdom = null;
			}
		});
	}

	@Override
	public void readPeriodicInputs() {
		for (SwerveModule swerveModule : mModules) {
			swerveModule.readPeriodicInputs();
		}

		mPeriodicIO.timestamp = Timer.getFPGATimestamp();
		mPeriodicIO.heading = mPigeon.getYaw();
		mPeriodicIO.pitch = mPigeon.getPitch();

		Twist2d twist_vel = Constants.SwerveConstants.kKinematics
				.toChassisSpeeds(getModuleStates())
				.toTwist2d();
		Translation2d translation_vel = new Translation2d(twist_vel.dx, twist_vel.dy);
		translation_vel = translation_vel.rotateBy(getHeading());
		mPeriodicIO.measured_velocity = new Twist2d(
				translation_vel.getTranslation().x(),
				translation_vel.getTranslation().y(),
				twist_vel.dtheta);
	}

	public synchronized void setTrajectory(TrajectoryIterator<TimedState<Pose2dWithMotion>> trajectory) {
		if (mMotionPlanner != null) {
			mOverrideTrajectory = false;
			mMotionPlanner.reset();
			mMotionPlanner.setTrajectory(trajectory);
			mControlState = DriveControlState.PATH_FOLLOWING;
		}
	}

	/**
	 * @param pid_enable Switches between using PID control or Pure Pursuit control to follow trajectories.
	 */
	public synchronized void setUsePIDControl(boolean pid_enable) {
		if (pid_enable) {
			mMotionPlanner.setFollowerType(FollowerType.PID);
		} else {
			mMotionPlanner.setFollowerType(FollowerType.PURE_PURSUIT);
		}
	}

	/**
	 * Generate and follow a trajectory from the robot-relative points provided.
	 * 
	 * @param relativeEndPos End position at relative to the current robot pose.
	 * @param targetHeading End heading relative to the field.
	 */
	public void setRobotCentricTrajectory(Translation2d relativeEndPos, Rotation2d targetHeading) {
		Translation2d end_position = getPose().getTranslation().translateBy(relativeEndPos);
		Rotation2d velocity_dir =
				(mWheelTracker.getMeasuredVelocity()).getTranslation().direction();
		List<Pose2d> waypoints = new ArrayList<>();
		List<Rotation2d> headings = new ArrayList<>();
		// Current state
		waypoints.add(new Pose2d(getPose().getTranslation(), velocity_dir));
		headings.add(getHeading());
		// Target state
		waypoints.add(new Pose2d(end_position, relativeEndPos.direction()));
		headings.add(targetHeading);
		Trajectory<TimedState<Pose2dWithMotion>> traj = mMotionPlanner.generateTrajectory(
				false, waypoints, headings, List.of(), Constants.SwerveConstants.maxAutoSpeed * 0.5, 2.54, 9.0);
		setTrajectory(new TrajectoryIterator<>(new TimedView<>(traj)));
	}

	/**
	 * Generate and follow a trajectory from the field-relative points provided.
	 * 
	 * @param fieldRelativeEndPos End position at relative to the field.
	 * @param targetHeading End heading relative to the field.
	 */
	public void setFieldCentricTrajectory(Translation2d fieldRelativeEndPos, Rotation2d targetHeading) {
		Translation2d robot_relative_end_pos =
				fieldRelativeEndPos.translateBy(getPose().getTranslation().inverse());

		Rotation2d velocity_dir = robot_relative_end_pos.direction();
		Translation2d velocity = mWheelTracker.getMeasuredVelocity();
		if (velocity.norm() > 0.2) {
			velocity_dir = velocity.direction();
		}

		List<Pose2d> waypoints = new ArrayList<>();
		List<Rotation2d> headings = new ArrayList<>();
		// Current state
		waypoints.add(new Pose2d(getPose().getTranslation(), velocity_dir));
		headings.add(getHeading());
		// Target state
		waypoints.add(new Pose2d(fieldRelativeEndPos, robot_relative_end_pos.direction()));
		headings.add(targetHeading);
		Trajectory<TimedState<Pose2dWithMotion>> traj = mMotionPlanner.generateTrajectory(
				false,
				waypoints,
				headings,
				List.of(),
				velocity.norm(),
				0.0,
				Constants.SwerveConstants.maxAutoSpeed * 0.5,
				2.54,
				9.0);
		LogUtil.recordTrajectory("OTF Traj", traj);
		setTrajectory(new TrajectoryIterator<>(new TimedView<>(traj)));
	}

	public synchronized boolean isDoneWithTrajectory() {
		if (mMotionPlanner == null || mControlState != DriveControlState.PATH_FOLLOWING) {
			return false;
		}
		return mMotionPlanner.isDone() || mOverrideTrajectory;
	}

	/**
	 * Exits trajectory following early.
	 * @param value Whether to override the current trajectory.
	 */
	public synchronized void overrideTrajectory(boolean value) {
		mOverrideTrajectory = value;
	}

	/**
	 * If in the Path Following state, updates the
	 * DriveMotionPlanner and PeriodicIO path setpoint/error.
	 */
	private void updatePathFollower() {
		if (mControlState == DriveControlState.PATH_FOLLOWING) {
			final double now = Timer.getFPGATimestamp();
			ChassisSpeeds output = mMotionPlanner.update(now, getPose(), mWheelTracker.getMeasuredVelocity());
			if (output != null) {
				mPeriodicIO.des_chassis_speeds = output;
			}

			mPeriodicIO.translational_error = mMotionPlanner.getTranslationalError();
			mPeriodicIO.heading_error = mMotionPlanner.getHeadingError();
			mPeriodicIO.path_setpoint = mMotionPlanner.getSetpoint();
		} else {
			DriverStation.reportError("Drive is not in path following state", false);
		}
	}

	/**
	 * Updates the wanted setpoint, including whether heading should
	 * be overridden to the tracking angle. Also includes
	 * updates for Path Following.
	 */
	private void updateSetpoint() {
		if (mControlState == DriveControlState.FORCE_ORIENT) return;

		Pose2d robot_pose_vel = new Pose2d(
				mPeriodicIO.des_chassis_speeds.vxMetersPerSecond * Constants.kLooperDt * 4.0,
				mPeriodicIO.des_chassis_speeds.vyMetersPerSecond * Constants.kLooperDt * 4.0,
				Rotation2d.fromRadians(
						mPeriodicIO.des_chassis_speeds.omegaRadiansPerSecond * Constants.kLooperDt * 4.0));
		Twist2d twist_vel = Pose2d.log(robot_pose_vel).scaled(1.0 / (4.0 * Constants.kLooperDt));

		ChassisSpeeds wanted_speeds;
		if (mOverrideHeading) {
			stabilizeHeading(mTrackingAngle);
			double new_omega = mHeadingController.update(mPeriodicIO.heading.getRadians(), Timer.getFPGATimestamp());
			ChassisSpeeds speeds = new ChassisSpeeds(twist_vel.dx, twist_vel.dy, new_omega);
			wanted_speeds = speeds;
		} else {
			wanted_speeds = new ChassisSpeeds(twist_vel.dx, twist_vel.dy, twist_vel.dtheta);
		}

		if (mControlState != DriveControlState.PATH_FOLLOWING) {
			// Limit rotational velocity
			wanted_speeds.omegaRadiansPerSecond = Math.signum(wanted_speeds.omegaRadiansPerSecond)
					* Math.min(mKinematicLimits.kMaxAngularVelocity, Math.abs(wanted_speeds.omegaRadiansPerSecond));

			// Limit translational velocity
			double velocity_magnitude = Math.hypot(
					mPeriodicIO.des_chassis_speeds.vxMetersPerSecond, mPeriodicIO.des_chassis_speeds.vyMetersPerSecond);
			if (velocity_magnitude > mKinematicLimits.kMaxDriveVelocity) {
				wanted_speeds.vxMetersPerSecond =
						(wanted_speeds.vxMetersPerSecond / velocity_magnitude) * mKinematicLimits.kMaxDriveVelocity;
				wanted_speeds.vyMetersPerSecond =
						(wanted_speeds.vyMetersPerSecond / velocity_magnitude) * mKinematicLimits.kMaxDriveVelocity;
			}

			SwerveModuleState[] prev_module_states =
					mPeriodicIO.des_module_states.clone(); // Get last setpoint to get differentials
			ChassisSpeeds prev_chassis_speeds = SwerveConstants.kKinematics.toChassisSpeeds(prev_module_states);
			SwerveModuleState[] target_module_states = SwerveConstants.kKinematics.toModuleStates(wanted_speeds);

			if (wanted_speeds.epsilonEquals(new ChassisSpeeds(), Util.kEpsilon)) {
				for (int i = 0; i < target_module_states.length; i++) {
					target_module_states[i].speedMetersPerSecond = 0.0;
					target_module_states[i].angle = prev_module_states[i].angle;
				}
			}

			double dx = wanted_speeds.vxMetersPerSecond - prev_chassis_speeds.vxMetersPerSecond;
			double dy = wanted_speeds.vyMetersPerSecond - prev_chassis_speeds.vyMetersPerSecond;
			double domega = wanted_speeds.omegaRadiansPerSecond - prev_chassis_speeds.omegaRadiansPerSecond;

			double max_velocity_step = mKinematicLimits.kMaxAccel * Constants.kLooperDt;
			double min_translational_scalar = 1.0;

			if (max_velocity_step < Double.MAX_VALUE * Constants.kLooperDt) {
				// Check X
				double x_norm = Math.abs(dx / max_velocity_step);
				min_translational_scalar = Math.min(min_translational_scalar, x_norm);

				// Check Y
				double y_norm = Math.abs(dy / max_velocity_step);
				min_translational_scalar = Math.min(min_translational_scalar, y_norm);

				min_translational_scalar *= max_velocity_step;
			}

			double max_omega_step = mKinematicLimits.kMaxAngularAccel * Constants.kLooperDt;
			double min_omega_scalar = 1.0;

			if (max_omega_step < Double.MAX_VALUE * Constants.kLooperDt) {
				double omega_norm = Math.abs(domega / max_omega_step);
				min_omega_scalar = Math.min(min_omega_scalar, omega_norm);

				min_omega_scalar *= max_omega_step;
			}

			SmartDashboard.putNumber("Accel", min_translational_scalar);

			wanted_speeds = new ChassisSpeeds(
					prev_chassis_speeds.vxMetersPerSecond + dx * min_translational_scalar,
					prev_chassis_speeds.vyMetersPerSecond + dy * min_translational_scalar,
					prev_chassis_speeds.omegaRadiansPerSecond + domega * min_omega_scalar);
		}

		SwerveModuleState[] real_module_setpoints = SwerveConstants.kKinematics.toModuleStates(wanted_speeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(real_module_setpoints, Constants.SwerveConstants.maxSpeed);

		mPeriodicIO.predicted_velocity =
				Pose2d.log(Pose2d.exp(wanted_speeds.toTwist2d()).rotateBy(getHeading()));
		mPeriodicIO.des_module_states = real_module_setpoints;
	}

	public void resetModulesToAbsolute() {
		for (SwerveModule module : mModules) {
			module.resetToAbsolute();
		}
	}

	public void zeroGyro() {
		zeroGyro(0.0);
	}

	public void zeroGyro(double reset_deg) {
		mPigeon.setYaw(reset_deg);
		enableFieldToOdom = null;
	}

	/**
	 * Configs if module drive motors should brake when commanded neutral output.
	 * @param brake Enable brake
	 */
	public void setNeutralBrake(boolean brake) {
		for (SwerveModule swerveModule : mModules) {
			swerveModule.setDriveNeutralBrake(brake);
		}
	}

	@Override
	public void writePeriodicOutputs() {
		for (int i = 0; i < mModules.length; i++) {
			if (mControlState == DriveControlState.OPEN_LOOP || mControlState == DriveControlState.HEADING_CONTROL) {
				mModules[i].setOpenLoop(mPeriodicIO.des_module_states[i]);
			} else if (mControlState == DriveControlState.PATH_FOLLOWING
					|| mControlState == DriveControlState.VELOCITY
					|| mControlState == DriveControlState.FORCE_ORIENT) {
				mModules[i].setVelocity(mPeriodicIO.des_module_states[i]);
			}
		}

		for (SwerveModule swerveModule : mModules) {
			swerveModule.writePeriodicOutputs();
		}
	}

	public SwerveModuleState[] getModuleStates() {
		SwerveModuleState[] states = new SwerveModuleState[4];
		for (SwerveModule mod : mModules) {
			states[mod.moduleNumber()] = mod.getState();
		}
		return states;
	}

	public SwerveModulePosition[] getModulePositions() {
		SwerveModulePosition[] states = new SwerveModulePosition[4];
		for (SwerveModule mod : mModules) {
			states[mod.moduleNumber()] = mod.getPosition();
		}
		return states;
	}

	public edu.wpi.first.math.kinematics.SwerveModulePosition[] getWpiModulePositions() {
		edu.wpi.first.math.kinematics.SwerveModulePosition[] states =
				new edu.wpi.first.math.kinematics.SwerveModulePosition[4];
		for (SwerveModule mod : mModules) {
			states[mod.moduleNumber()] = mod.getWpiPosition();
		}
		return states;
	}

	public Pose2d getPose() {
		return RobotState.getInstance().getLatestFieldToVehicle();
	}

	public void resetOdometry(Pose2d pose) {
		odometryReset = true;
		Pose2d wanted_pose = pose;
		mWheelTracker.resetPose(wanted_pose);
	}

	public void resetOdometry(edu.wpi.first.math.geometry.Pose2d pose) {
		resetOdometry(Util.to254Pose(pose));
	}

	public boolean readyForAuto() {
		return odometryReset;
	}

	public Rotation2d getHeading() {
		return mPigeon.getYaw();
	}

	public DriveMotionPlanner getMotionPlanner() {
		return mMotionPlanner;
	}

	public KinematicLimits getKinematicLimits() {
		return mKinematicLimits;
	}

	public static class PeriodicIO {
		// Inputs/Desired States
		double timestamp;
		ChassisSpeeds des_chassis_speeds = new ChassisSpeeds(0.0, 0.0, 0.0);
		Twist2d measured_velocity = Twist2d.identity();
		Rotation2d heading = new Rotation2d();
		Rotation2d pitch = new Rotation2d();

		// Outputs
		SwerveModuleState[] des_module_states = new SwerveModuleState[] {
			new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()
		};
		Twist2d predicted_velocity = Twist2d.identity();
		Translation2d translational_error = Translation2d.identity();
		Rotation2d heading_error = Rotation2d.identity();
		TimedState<Pose2dWithMotion> path_setpoint = new TimedState<Pose2dWithMotion>(Pose2dWithMotion.identity());
		Rotation2d heading_setpoint = new Rotation2d();
	}

	@Override
	public void outputTelemetry() {
		if (Constants.disableExtraTelemetry) {
			return;
		}

		if (enableFieldToOdom == null && RobotState.getInstance().getHasRecievedVisionUpdate()) {
			enableFieldToOdom = RobotState.getInstance().getLatestFieldToOdom();
		}

		if (enableFieldToOdom != null) {
			Pose2d latestOdomToVehicle =
					RobotState.getInstance().getLatestOdomToVehicle().getValue();
			latestOdomToVehicle = Pose2d.fromTranslation(enableFieldToOdom).transformBy(latestOdomToVehicle);
			LogUtil.recordPose2d("Odometry Pose", latestOdomToVehicle);
		}

		for (SwerveModule module : mModules) {
			module.outputTelemetry();
		}
		SmartDashboard.putString("Drive Control State", mControlState.toString());
		SmartDashboard.putBoolean("Is done with trajectory", isDoneWithTrajectory());

		SmartDashboard.putNumber("Target omega", mPeriodicIO.des_chassis_speeds.omegaRadiansPerSecond);
		SmartDashboard.putNumber(
				"Real omega",
				Constants.SwerveConstants.kKinematics.toChassisSpeeds(getModuleStates()).omegaRadiansPerSecond);
		LogUtil.recordPose2d("Drive Pose", mWheelTracker.getRobotPose());
		LogUtil.recordPose2d("Fused Pose", RobotState.getInstance().getLatestFieldToVehicle());

		SmartDashboard.putBoolean("Target tracking", mOverrideHeading);

		SmartDashboard.putNumber(
				"Drive Velo",
				Math.hypot(
						Constants.SwerveConstants.kKinematics.toChassisSpeeds(getModuleStates()).vxMetersPerSecond,
						Constants.SwerveConstants.kKinematics.toChassisSpeeds(getModuleStates()).vyMetersPerSecond));
	}

	public DriveControlState getControlState() {
		return mControlState;
	}

	@Override
	public void stop() {
		mPeriodicIO.des_chassis_speeds = new ChassisSpeeds();
		mControlState = DriveControlState.OPEN_LOOP;
	}

	@Override
	public boolean checkSystem() {
		return false;
	}

	public static class KinematicLimits {
		public double kMaxDriveVelocity = Constants.SwerveConstants.maxSpeed; // m/s
		public double kMaxAccel = Double.MAX_VALUE; // m/s^2
		public double kMaxAngularVelocity = Constants.SwerveConstants.maxAngularVelocity; // rad/s
		public double kMaxAngularAccel = Double.MAX_VALUE; // rad/s^2
	}
}
