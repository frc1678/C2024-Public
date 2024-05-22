package com.team1678.lib.swerve;

import com.team1678.frc2024.Constants;
import com.team1678.lib.logger.LogUtil;
import com.team254.lib.control.ErrorTracker;
import com.team254.lib.control.Lookahead;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithMotion;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.trajectory.DistanceView;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.TrajectorySamplePoint;
import com.team254.lib.trajectory.TrajectoryUtil;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.trajectory.timing.TimingConstraint;
import com.team254.lib.trajectory.timing.TimingUtil;
import com.team254.lib.util.CSVWritable;
import com.team254.lib.util.SynchronousPIDF;
import com.team254.lib.util.Util;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class DriveMotionPlanner implements CSVWritable {
	private static final double kMaxDx = 0.0127; // m
	private static final double kMaxDy = 0.0127; // m
	private static final double kMaxDTheta = Math.toRadians(1.0);

	// Pure Pursuit Constants
	public static final double kPathLookaheadTime = 0.1; // From 1323 (2019)
	public static final double kPathMinLookaheadDistance = 0.3; // From 1323 (2019)
	public static final double kAdaptivePathMinLookaheadDistance = 0.15;
	public static final double kAdaptivePathMaxLookaheadDistance = 0.61;
	public static final double kAdaptiveErrorLookaheadCoefficient = 0.01;

	public enum FollowerType {
		FEEDFORWARD_ONLY,
		PID,
		PURE_PURSUIT,
		RAMSETE
	}

	FollowerType mFollowerType = FollowerType.PURE_PURSUIT;

	public void setFollowerType(FollowerType type) {
		mFollowerType = type;
	}

	private double defaultCook = 0.5;
	private boolean useDefaultCook = true;

	public void setDefaultCook(double new_value) {
		defaultCook = new_value;
	}

	TrajectoryIterator<TimedState<Pose2dWithMotion>> mCurrentTrajectory;
	boolean mIsReversed = false;
	double mLastTime = Double.POSITIVE_INFINITY;
	public TimedState<Pose2dWithMotion> mLastSetpoint = null;
	public TimedState<Pose2dWithMotion> mSetpoint = new TimedState<>(Pose2dWithMotion.identity());
	Pose2d mError = Pose2d.identity();

	ErrorTracker mErrorTracker = new ErrorTracker(15 * 100);
	Translation2d mTranslationalError = Translation2d.identity();
	Rotation2d mPrevHeadingError = Rotation2d.identity();
	Pose2d mCurrentState = Pose2d.identity();

	double mCurrentTrajectoryLength = 0.0;
	double mTotalTime = Double.POSITIVE_INFINITY;
	double mStartTime = Double.POSITIVE_INFINITY;
	ChassisSpeeds mOutput = new ChassisSpeeds();

	Lookahead mSpeedLookahead = null;

	// PID controllers for path following
	SynchronousPIDF mXPIDF;
	SynchronousPIDF mYPIDF;
	SynchronousPIDF mHeadingPIDF;

	double mDt = 0.0;

	public DriveMotionPlanner() {}

	public void setTrajectory(final TrajectoryIterator<TimedState<Pose2dWithMotion>> trajectory) {
		mCurrentTrajectory = trajectory;
		mSetpoint = trajectory.getState();
		mLastSetpoint = null;
		useDefaultCook = true;
		mSpeedLookahead = new Lookahead(
				kAdaptivePathMinLookaheadDistance,
				kAdaptivePathMaxLookaheadDistance,
				0.0,
				Constants.SwerveConstants.maxAutoSpeed);
		mCurrentTrajectoryLength =
				mCurrentTrajectory.trajectory().getLastPoint().state().t();
		for (int i = 0; i < trajectory.trajectory().length(); ++i) {
			if (trajectory.trajectory().getPoint(i).state().velocity() > Util.kEpsilon) {
				mIsReversed = false;
				break;
			} else if (trajectory.trajectory().getPoint(i).state().velocity() < -Util.kEpsilon) {
				mIsReversed = true;
				break;
			}
		}
	}

	public void reset() {
		mErrorTracker.reset();
		mTranslationalError = Translation2d.identity();
		mPrevHeadingError = Rotation2d.identity();
		mLastSetpoint = null;
		mOutput = new ChassisSpeeds();
		mLastTime = Double.POSITIVE_INFINITY;
	}

	public Trajectory<TimedState<Pose2dWithMotion>> generateTrajectory(
			boolean reversed,
			final List<Pose2d> waypoints,
			final List<Rotation2d> headings,
			final List<TimingConstraint<Pose2dWithMotion>> constraints,
			double max_vel, // m/s
			double max_accel, // m/s^2
			double max_voltage) {
		return generateTrajectory(
				reversed, waypoints, headings, constraints, 0.0, 0.0, max_vel, max_accel, max_voltage);
	}

	public Trajectory<TimedState<Pose2dWithMotion>> generateTrajectory(
			boolean reversed,
			final List<Pose2d> waypoints,
			final List<Rotation2d> headings,
			final List<TimingConstraint<Pose2dWithMotion>> constraints,
			double start_vel,
			double end_vel,
			double max_vel, // m/s
			double max_accel, // m/s^2
			double max_voltage) {
		List<Pose2d> waypoints_maybe_flipped = waypoints;
		List<Rotation2d> headings_maybe_flipped = headings;
		final Pose2d flip = Pose2d.fromRotation(new Rotation2d(-1, 0, false));
		if (reversed) {
			waypoints_maybe_flipped = new ArrayList<>(waypoints.size());
			headings_maybe_flipped = new ArrayList<>(headings.size());
			for (int i = 0; i < waypoints.size(); ++i) {
				waypoints_maybe_flipped.add(waypoints.get(i).transformBy(flip));
				headings_maybe_flipped.add(headings.get(i).rotateBy(flip.getRotation()));
			}
		}

		// Create a trajectory from splines.
		Trajectory<Pose2dWithMotion> trajectory = TrajectoryUtil.trajectoryFromWaypointsAndHeadings(
				waypoints_maybe_flipped, headings_maybe_flipped, kMaxDx, kMaxDy, kMaxDTheta);

		if (reversed) {
			List<Pose2dWithMotion> flipped_points = new ArrayList<>(trajectory.length());
			for (int i = 0; i < trajectory.length(); ++i) {
				flipped_points.add(new Pose2dWithMotion(
						trajectory.getPoint(i).state().getPose().transformBy(flip),
						-trajectory.getPoint(i).state().getCurvature(),
						trajectory.getPoint(i).state().getDCurvatureDs()));
			}
			trajectory = new Trajectory<>(flipped_points);
		}

		// No constraints? :(
		List<TimingConstraint<Pose2dWithMotion>> all_constraints = new ArrayList<>();
		// all_constraints.add(drive_constraints);
		// all_constraints.add(yaw_constraint);
		// all_constraints.add(centripetal_accel_constraint);
		if (constraints != null) {
			all_constraints.addAll(constraints);
		}

		// Generate the timed trajectory.
		Trajectory<TimedState<Pose2dWithMotion>> timed_trajectory = TimingUtil.timeParameterizeTrajectory(
				reversed,
				new DistanceView<>(trajectory),
				kMaxDx,
				all_constraints,
				start_vel,
				end_vel,
				max_vel,
				max_accel);
		return timed_trajectory;
	}

	@Override
	public String toCSV() {
		DecimalFormat fmt = new DecimalFormat("#0.000");
		String ret = "";
		ret += fmt.format(mOutput.vxMetersPerSecond + ",");
		ret += fmt.format(mOutput.vyMetersPerSecond + ",");
		ret += fmt.format(mOutput.omegaRadiansPerSecond + ",");
		return ret + mSetpoint.toCSV();
	}

	protected ChassisSpeeds updateRamsete(
			TimedState<Pose2dWithMotion> fieldToGoal, Pose2d fieldToRobot, Twist2d currentVelocity) {
		// Implements eqn. 5.12 from https://www.dis.uniroma1.it/~labrob/pub/papers/Ramsete01.pdf
		final double kBeta = 2.0; // >0.
		final double kZeta = 0.7; // Damping coefficient, [0, 1].

		// Convert from current velocity into course.
		Optional<Rotation2d> maybe_field_to_course = Optional.empty();
		Optional<Rotation2d> maybe_robot_to_course = currentVelocity.getCourse();
		if (maybe_robot_to_course.isPresent()) {
			// Course is robot_to_course, we want to be field_to_course.
			// field_to_course = field_to_robot * robot_to_course
			maybe_field_to_course = Optional.of(fieldToRobot.getRotation().rotateBy(maybe_robot_to_course.get()));
		}

		// Convert goal into a desired course (in robot frame).
		double goal_linear_velocity = fieldToGoal.velocity();
		double goal_angular_velocity =
				goal_linear_velocity * fieldToGoal.state().getCurvature();
		Optional<Rotation2d> maybe_field_to_goal = fieldToGoal.state().getCourse();

		// Deal with lack of course data by always being optimistic.
		if (maybe_field_to_course.isEmpty()) {
			maybe_field_to_course = maybe_field_to_goal;
		}
		if (maybe_field_to_goal.isEmpty()) {
			maybe_field_to_goal = maybe_field_to_course;
		}
		if (maybe_field_to_goal.isEmpty() && maybe_field_to_course.isEmpty()) {
			// Course doesn't matter.
			maybe_field_to_course = maybe_field_to_goal = Optional.of(Rotation2d.kIdentity);
		}
		Rotation2d field_to_course = maybe_field_to_course.get();
		Rotation2d robot_to_course = fieldToRobot.getRotation().inverse().rotateBy(field_to_course);

		// Convert goal course to be relative to current course.
		// course_to_goal = course_to_field * field_to_goal
		Rotation2d course_to_goal = field_to_course.inverse().rotateBy(maybe_field_to_goal.get());

		// Rotate error to be aligned to current course.
		// Error is in robot (heading) frame. Need to rotate it to be in course frame.
		// course_to_error = robot_to_course.inverse() * robot_to_error
		Translation2d linear_error_course_relative =
				Pose2d.fromRotation(robot_to_course).transformBy(mError).getTranslation();

		// Compute time-varying gain parameter.
		final double k = 2.0
				* kZeta
				* Math.sqrt(kBeta * goal_linear_velocity * goal_linear_velocity
						+ goal_angular_velocity * goal_angular_velocity);

		// Compute error components.
		final double angle_error_rads = course_to_goal.getRadians();
		final double sin_x_over_x =
				Util.epsilonEquals(angle_error_rads, 0.0, 1E-2) ? 1.0 : course_to_goal.sin() / angle_error_rads;
		double adjusted_linear_velocity =
				goal_linear_velocity * course_to_goal.cos() + k * linear_error_course_relative.x();
		double adjusted_angular_velocity = goal_angular_velocity
				+ k * angle_error_rads
				+ goal_linear_velocity * kBeta * sin_x_over_x * linear_error_course_relative.y();

		final double kThetaKp = 5.0; // Units are rad/s per rad of error.
		double heading_rate = goal_linear_velocity * fieldToGoal.state().getHeadingRate()
				+ kThetaKp * mError.getRotation().getRadians();

		// Create a course-relative Twist2d.
		Twist2d adjusted_course_relative_velocity =
				new Twist2d(adjusted_linear_velocity, 0.0, adjusted_angular_velocity - heading_rate);
		// See where that takes us in one dt.
		final double kNominalDt = Constants.kLooperDt;
		Pose2d adjusted_course_to_goal = Pose2d.exp(adjusted_course_relative_velocity.scaled(kNominalDt));

		// Now rotate to be robot-relative.
		// robot_to_goal = robot_to_course * course_to_goal
		Translation2d adjusted_robot_to_goal = Pose2d.fromRotation(robot_to_course)
				.transformBy(adjusted_course_to_goal)
				.getTranslation()
				.scale(1.0 / kNominalDt);

		return new ChassisSpeeds(adjusted_robot_to_goal.x(), adjusted_robot_to_goal.y(), heading_rate);
	}

	protected ChassisSpeeds updatePIDChassis(ChassisSpeeds chassisSpeeds) {
		// Feedback on longitudinal error (distance).
		final double kPathk =
				2.4; // 2.4;/* * Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)*/;//0.15;
		final double kPathKTheta = 3.0;

		Twist2d pid_error = Pose2d.log(mError);

		chassisSpeeds.vxMetersPerSecond = chassisSpeeds.vxMetersPerSecond + kPathk * pid_error.dx;
		chassisSpeeds.vyMetersPerSecond = chassisSpeeds.vyMetersPerSecond + kPathk * pid_error.dy;
		chassisSpeeds.omegaRadiansPerSecond = chassisSpeeds.omegaRadiansPerSecond + kPathKTheta * pid_error.dtheta;
		return chassisSpeeds;
	}

	protected ChassisSpeeds updatePurePursuit(Pose2d current_state, double feedforwardOmegaRadiansPerSecond) {
		double lookahead_time = kPathLookaheadTime;
		final double kLookaheadSearchDt = 0.01;
		TimedState<Pose2dWithMotion> lookahead_state =
				mCurrentTrajectory.preview(lookahead_time).state();
		double actual_lookahead_distance = mSetpoint.state().distance(lookahead_state.state());
		double adaptive_lookahead_distance = mSpeedLookahead.getLookaheadForSpeed(mSetpoint.velocity())
				+ kAdaptiveErrorLookaheadCoefficient * mError.getTranslation().norm();
		SmartDashboard.putNumber("PurePursuit/Error", mError.getTranslation().norm());
		// Find the Point on the Trajectory that is Lookahead Distance Away
		while (actual_lookahead_distance < adaptive_lookahead_distance
				&& mCurrentTrajectory.getRemainingProgress() > lookahead_time) {
			lookahead_time += kLookaheadSearchDt;
			lookahead_state = mCurrentTrajectory.preview(lookahead_time).state();
			actual_lookahead_distance = mSetpoint.state().distance(lookahead_state.state());
		}

		// If the Lookahead Point's Distance is less than the Lookahead Distance transform it so it is the lookahead
		// distance away
		if (actual_lookahead_distance < adaptive_lookahead_distance) {
			lookahead_state = new TimedState<>(
					new Pose2dWithMotion(
							lookahead_state
									.state()
									.getPose()
									.transformBy(Pose2d.fromTranslation(new Translation2d(
											(mIsReversed ? -1.0 : 1.0)
													* (kPathMinLookaheadDistance - actual_lookahead_distance),
											0.0))),
							0.0),
					lookahead_state.t(),
					lookahead_state.velocity(),
					lookahead_state.acceleration());
		}
		SmartDashboard.putNumber("PurePursuit/ActualLookaheadDist", actual_lookahead_distance);
		SmartDashboard.putNumber("PurePursuit/AdaptiveLookahead", adaptive_lookahead_distance);
		LogUtil.recordPose2d(
				"PurePursuit/LookaheadState", lookahead_state.state().getPose());
		SmartDashboard.putNumber("PurePursuit/RemainingProgress", mCurrentTrajectory.getRemainingProgress());
		SmartDashboard.putNumber("PurePursuit/PathVelocity", lookahead_state.velocity());

		if (lookahead_state.velocity() == 0.0) {
			mCurrentTrajectory.advance(Double.POSITIVE_INFINITY);
			return new ChassisSpeeds();
		}

		// Find the vector between robot's current position and the lookahead state
		Translation2d lookaheadTranslation = new Translation2d(
				current_state.getTranslation(), lookahead_state.state().getTranslation());

		// Set the steering direction as the direction of the vector
		Rotation2d steeringDirection = lookaheadTranslation.direction();

		// Convert from field-relative steering direction to robot-relative
		steeringDirection = steeringDirection.rotateBy(current_state.inverse().getRotation());

		// Use the Velocity Feedforward of the Closest Point on the Trajectory
		double normalizedSpeed = Math.abs(mSetpoint.velocity()) / Constants.SwerveConstants.maxAutoSpeed;

		// The Default Cook is the minimum speed to use. So if a feedforward speed is less than defaultCook, the robot
		// will drive at the defaultCook speed
		if (normalizedSpeed > defaultCook || mSetpoint.t() > (mCurrentTrajectoryLength / 2.0)) {
			useDefaultCook = false;
		}
		if (useDefaultCook) {
			normalizedSpeed = defaultCook;
		}

		SmartDashboard.putNumber("PurePursuit/NormalizedSpeed", normalizedSpeed);

		// Convert the Polar Coordinate (speed, direction) into a Rectangular Coordinate (Vx, Vy) in Robot Frame
		final Translation2d steeringVector =
				new Translation2d(steeringDirection.cos() * normalizedSpeed, steeringDirection.sin() * normalizedSpeed);
		ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
				steeringVector.x() * Constants.SwerveConstants.maxAutoSpeed,
				steeringVector.y() * Constants.SwerveConstants.maxAutoSpeed,
				feedforwardOmegaRadiansPerSecond);

		// Use the PD-Controller for To Follow the Time-Parametrized Heading
		final double kThetakP = 3.5;
		final double kThetakD = 0.0;
		final double kPositionkP = 2.0;

		chassisSpeeds.vxMetersPerSecond = chassisSpeeds.vxMetersPerSecond
				+ kPositionkP * mError.getTranslation().x();
		chassisSpeeds.vyMetersPerSecond = chassisSpeeds.vyMetersPerSecond
				+ kPositionkP * mError.getTranslation().y();
		chassisSpeeds.omegaRadiansPerSecond = chassisSpeeds.omegaRadiansPerSecond
				+ (kThetakP * mError.getRotation().getRadians())
				+ kThetakD * ((mError.getRotation().getRadians() - mPrevHeadingError.getRadians()) / mDt);
		return chassisSpeeds;
	}

	public ChassisSpeeds update(double timestamp, Pose2d current_state, Translation2d current_velocity) {
		if (mCurrentTrajectory == null) return null;

		if (!Double.isFinite(mLastTime)) mLastTime = timestamp;
		mDt = timestamp - mLastTime;
		mLastTime = timestamp;
		TrajectorySamplePoint<TimedState<Pose2dWithMotion>> sample_point;
		mCurrentState = current_state;

		if (!isDone()) {
			// Compute error in robot frame
			mPrevHeadingError = mError.getRotation();
			mError = current_state.inverse().transformBy(mSetpoint.state().getPose());
			mErrorTracker.addObservation(mError);

			if (mFollowerType == FollowerType.FEEDFORWARD_ONLY) {
				sample_point = mCurrentTrajectory.advance(mDt);
				// RobotState.getInstance().setDisplaySetpointPose(Pose2d.fromTranslation(RobotState.getInstance().getFieldToOdom(timestamp)).transformBy(sample_point.state().state().getPose()));
				mSetpoint = sample_point.state();

				final double velocity_m = mSetpoint.velocity();
				// Field relative
				var course = mSetpoint.state().getCourse();
				Rotation2d motion_direction = course.isPresent() ? course.get() : Rotation2d.identity();
				// Adjust course by ACTUAL heading rather than planned to decouple heading and translation errors.
				motion_direction = current_state.getRotation().inverse().rotateBy(motion_direction);

				mOutput = new ChassisSpeeds(
						motion_direction.cos() * velocity_m,
						motion_direction.sin() * velocity_m,
						// Need unit conversion because Pose2dWithMotion heading rate is per unit distance.
						velocity_m * mSetpoint.state().getHeadingRate());
			} else if (mFollowerType == FollowerType.RAMSETE) {
				sample_point = mCurrentTrajectory.advance(mDt);
				// RobotState.getInstance().setDisplaySetpointPose(Pose2d.fromTranslation(RobotState.getInstance().getFieldToOdom(timestamp)).transformBy(sample_point.state().state().getPose()));
				mSetpoint = sample_point.state();
				// mOutput = updateRamsete(sample_point.state(), current_state, current_velocity);
			} else if (mFollowerType == FollowerType.PID) {
				sample_point = mCurrentTrajectory.advance(mDt);
				// RobotState.getInstance().setDisplaySetpointPose(Pose2d.fromTranslation(RobotState.getInstance().getFieldToOdom(timestamp)).transformBy(sample_point.state().state().getPose()));
				mSetpoint = sample_point.state();

				final double velocity_m = mSetpoint.velocity();
				// Field relative
				var course = mSetpoint.state().getCourse();
				Rotation2d motion_direction = course.isPresent() ? course.get() : Rotation2d.identity();
				// Adjust course by ACTUAL heading rather than planned to decouple heading and translation errors.
				motion_direction = current_state.getRotation().inverse().rotateBy(motion_direction);

				var chassis_speeds = new ChassisSpeeds(
						motion_direction.cos() * velocity_m,
						motion_direction.sin() * velocity_m,
						// Need unit conversion because Pose2dWithMotion heading rate is per unit distance.
						velocity_m * mSetpoint.state().getHeadingRate());
				// PID is in robot frame
				mOutput = updatePIDChassis(chassis_speeds);
			} else if (mFollowerType == FollowerType.PURE_PURSUIT) {
				double searchStepSize = 1.0;
				double previewQuantity = 0.0;
				double searchDirection = 1.0;
				double forwardDistance = distance(current_state, previewQuantity + searchStepSize);
				double reverseDistance = distance(current_state, previewQuantity - searchStepSize);
				searchDirection = Math.signum(reverseDistance - forwardDistance);
				while (searchStepSize > 0.001) {
					SmartDashboard.putNumber("PurePursuit/PreviewDist", distance(current_state, previewQuantity));
					if (Util.epsilonEquals(distance(current_state, previewQuantity), 0.0, 0.0003937)) break;
					while (
					/* next point is closer than current point */ distance(
									current_state, previewQuantity + searchStepSize * searchDirection)
							< distance(current_state, previewQuantity)) {
						/* move to next point */
						previewQuantity += searchStepSize * searchDirection;
					}
					searchStepSize /= 10.0;
					searchDirection *= -1;
				}
				SmartDashboard.putNumber("PurePursuit/PreviewQtd", previewQuantity);
				sample_point = mCurrentTrajectory.advance(previewQuantity);
				// RobotState.getInstance().setDisplaySetpointPose(Pose2d.fromTranslation(RobotState.getInstance().getFieldToOdom(timestamp)).transformBy(sample_point.state().state().getPose()));
				mSetpoint = sample_point.state();
				mOutput = updatePurePursuit(current_state, 0.0);
			}
		} else {
			if (mCurrentTrajectory.trajectory().getLastPoint().state().velocity() == 0.0) {
				mOutput = new ChassisSpeeds();
			}
		}

		return mOutput;
	}

	public boolean isDone() {
		return mCurrentTrajectory != null && mCurrentTrajectory.isDone();
	}

	public synchronized Translation2d getTranslationalError() {
		return new Translation2d(
				mError.getTranslation().x(), mError.getTranslation().y());
	}

	public synchronized Rotation2d getHeadingError() {
		return mError.getRotation();
	}

	private double distance(Pose2d current_state, double additional_progress) {
		return mCurrentTrajectory
				.preview(additional_progress)
				.state()
				.state()
				.getPose()
				.distance(current_state);
	}

	public synchronized TimedState<Pose2dWithMotion> getSetpoint() {
		return mSetpoint;
	}

	public synchronized ErrorTracker getErrorTracker() {
		return mErrorTracker;
	}
}
