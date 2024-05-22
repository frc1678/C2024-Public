package com.team1678.frc2024;

import com.team1678.frc2024.subsystems.vision.VisionPoseAcceptor;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.InterpolatingTreeMap;
import com.team254.lib.util.MovingAverageTwist2d;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.ExtendedKalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Map;
import java.util.Optional;

public class RobotState {
	private static RobotState mInstance;

	public static RobotState getInstance() {
		if (mInstance == null) {
			mInstance = new RobotState();
		}
		return mInstance;
	}

	private static final int kObservationBufferSize = 50;
	private static final Matrix<N2, N1> kStateStdDevs = VecBuilder.fill(Math.pow(0.05, 1), Math.pow(0.05, 1)); // drive
	private static final Matrix<N2, N1> kLocalMeasurementStdDevs = VecBuilder.fill(
			Math.pow(0.02, 1), // vision
			Math.pow(0.02, 1));
	private Optional<VisionUpdate> mLatestVisionUpdate;

	private Optional<Translation2d> initial_field_to_odom = Optional.empty();
	private InterpolatingTreeMap<InterpolatingDouble, Pose2d> odometry_to_vehicle;
	private InterpolatingTreeMap<InterpolatingDouble, Translation2d> field_to_odometry;
	private ExtendedKalmanFilter<N2, N2, N2> mKalmanFilter;
	private VisionPoseAcceptor mPoseAcceptor;

	private Twist2d vehicle_velocity_measured;
	private Twist2d vehicle_velocity_predicted;
	private MovingAverageTwist2d vehicle_velocity_measured_filtered;

	private boolean mHasRecievedVisionUpdate = false;
	private boolean mIsInAuto = false;

	public RobotState() {
		reset(0.0, Pose2d.identity());
	}

	/**
	 * Clears pose history and sets odometry pose as truth.
	 *
	 * @param now                     Current timestamp.
	 * @param initial_odom_to_vehicle Initial pose of the robot.
	 */
	public synchronized void reset(double now, Pose2d initial_odom_to_vehicle) {
		odometry_to_vehicle = new InterpolatingTreeMap<>(kObservationBufferSize);
		odometry_to_vehicle.put(new InterpolatingDouble(now), initial_odom_to_vehicle);
		field_to_odometry = new InterpolatingTreeMap<>(kObservationBufferSize);
		field_to_odometry.put(new InterpolatingDouble(now), getInitialFieldToOdom());
		vehicle_velocity_measured = Twist2d.identity();
		vehicle_velocity_predicted = Twist2d.identity();
		vehicle_velocity_measured_filtered = new MovingAverageTwist2d(25);
		mLatestVisionUpdate = Optional.empty();
		mPoseAcceptor = new VisionPoseAcceptor();
	}

	/**
	 * Reconstructs Kalman Filter.
	 */
	public synchronized void resetKalman() {
		mKalmanFilter = new ExtendedKalmanFilter<N2, N2, N2>(
				Nat.N2(), // Dimensions of output (x, y)
				Nat.N2(), // Dimensions of predicted error shift (dx, dy) (always 0)
				Nat.N2(), // Dimensions of vision (x, y)
				(x, u) -> u, // The derivative of the output is predicted shift (always 0)
				(x, u) -> x, // The output is position (x, y)
				kStateStdDevs, // Standard deviation of position (uncertainty propagation with no vision)
				kLocalMeasurementStdDevs, // Standard deviation of vision measurements
				Constants.kLooperDt);
	}

	/**
	 * Adds new odometry pose update.
	 *
	 * @param now                Timestamp of observation.
	 * @param odometry_pose      Reported pose from odometry.
	 * @param measured_velocity  Measured field-relative velocity.
	 * @param predicted_velocity Predicted field-relative velocity (usually swerve
	 *                           setpoint).
	 */
	public synchronized void addOdometryUpdate(
			double now, Pose2d odometry_pose, Twist2d measured_velocity, Twist2d predicted_velocity) {
		odometry_to_vehicle.put(new InterpolatingDouble(now), odometry_pose);
		mKalmanFilter.predict(
				VecBuilder.fill(0.0, 0.0), Constants.kLooperDt); // Propagate error of current vision prediction
		vehicle_velocity_measured = measured_velocity;
		vehicle_velocity_measured_filtered.add(measured_velocity);
		vehicle_velocity_predicted = predicted_velocity;
	}

	/**
	 * Adds new vision pose update.
	 *
	 * @param update Info about vision update.
	 */
	public synchronized void addVisionUpdate(VisionUpdate update) {
		// If it's the first update don't do filtering
		if (!mLatestVisionUpdate.isPresent() || initial_field_to_odom.isEmpty()) {
			double vision_timestamp = update.timestamp;
			Pose2d proximate_dt_pose = odometry_to_vehicle.getInterpolated(new InterpolatingDouble(vision_timestamp));
			Translation2d field_to_vision = update.field_to_camera.translateBy(update.getRobotToCamera()
					.rotateBy(getLatestOdomToVehicle().getValue().getRotation())
					.inverse());
			Translation2d odom_to_vehicle_translation = proximate_dt_pose.getTranslation();
			Translation2d field_to_odom = field_to_vision.translateBy(odom_to_vehicle_translation.inverse());
			field_to_odometry.put(new InterpolatingDouble(vision_timestamp), field_to_odom);
			initial_field_to_odom = Optional.of(field_to_odometry.lastEntry().getValue());
			mKalmanFilter.setXhat(0, field_to_odom.x());
			mKalmanFilter.setXhat(1, field_to_odom.y());
			mLatestVisionUpdate = Optional.ofNullable(update);
		} else {
			double vision_timestamp = mLatestVisionUpdate.get().timestamp;
			Pose2d proximate_dt_pose = odometry_to_vehicle.getInterpolated(new InterpolatingDouble(vision_timestamp));
			mLatestVisionUpdate = Optional.ofNullable(update);
			Translation2d field_to_vision = mLatestVisionUpdate
					.get()
					.field_to_camera
					.translateBy(mLatestVisionUpdate
							.get()
							.getRobotToCamera()
							.rotateBy(proximate_dt_pose.getRotation())
							.inverse());

			if (mPoseAcceptor.shouldAcceptVision(
					vision_timestamp,
					new Pose2d(field_to_vision, new Rotation2d()),
					getLatestFieldToVehicle(),
					vehicle_velocity_measured,
					mIsInAuto)) {
				Translation2d field_to_odom = field_to_vision.translateBy(
						proximate_dt_pose.getTranslation().inverse());
				try {
					Vector<N2> stdevs = VecBuilder.fill(Math.pow(update.xy_stdev, 1), Math.pow(update.xy_stdev, 1));
					mKalmanFilter.correct(
							VecBuilder.fill(0.0, 0.0),
							VecBuilder.fill(
									field_to_odom.getTranslation().x(),
									field_to_odom.getTranslation().y()),
							StateSpaceUtil.makeCovarianceMatrix(Nat.N2(), stdevs));
					field_to_odometry.put(
							new InterpolatingDouble(vision_timestamp),
							new Translation2d(mKalmanFilter.getXhat(0), mKalmanFilter.getXhat(1)));
					if (!getHasRecievedVisionUpdate()) {
						mHasRecievedVisionUpdate = true;
					}
				} catch (Exception e) {
					DriverStation.reportError(update.xy_stdev + "//QR Decomposition failed: ", e.getStackTrace());
				}
			}
		}
	}

	/**
	 * Gets initial odometry error. Odometry initializes to the origin, while the
	 * robot starts at an unknown position on the field.
	 *
	 * @return Initial odometry error translation.
	 */
	public synchronized Translation2d getInitialFieldToOdom() {
		if (initial_field_to_odom.isEmpty()) return Translation2d.identity();
		return initial_field_to_odom.get();
	}

	/**
	 * @return Latest field relative robot pose.
	 */
	public synchronized Pose2d getLatestFieldToVehicle() {
		Pose2d odomToVehicle = getLatestOdomToVehicle().getValue();

		Translation2d fieldToOdom = getLatestFieldToOdom();
		return new Pose2d(fieldToOdom.translateBy(odomToVehicle.getTranslation()), odomToVehicle.getRotation());
	}

	/**
	 * Gets field relative robot pose from history. Linearly interpolates between
	 * gaps.
	 *
	 * @param timestamp Timestamp to look up.
	 * @return Field relative robot pose at timestamp.
	 */
	public synchronized Pose2d getFieldToVehicle(double timestamp) {
		Pose2d odomToVehicle = getOdomToVehicle(timestamp);

		Translation2d fieldToOdom = getFieldToOdom(timestamp);
		return new Pose2d(fieldToOdom.translateBy(odomToVehicle.getTranslation()), odomToVehicle.getRotation());
	}

	/**
	 * Gets interpolated robot pose using predicted robot velocity from latest
	 * odometry update.
	 *
	 * @param lookahead_time Scalar for predicted velocity.
	 * @return Predcited robot pose at lookahead time.
	 */
	public synchronized Pose2d getPredictedFieldToVehicle(double lookahead_time) {
		Pose2d odomToVehicle = getPredictedOdomToVehicle(lookahead_time);

		Translation2d fieldToOdom = getLatestFieldToOdom();
		return new Pose2d(fieldToOdom.translateBy(odomToVehicle.getTranslation()), odomToVehicle.getRotation());
	}

	/**
	 * @return Latest odometry pose.
	 */
	public synchronized Map.Entry<InterpolatingDouble, Pose2d> getLatestOdomToVehicle() {
		return odometry_to_vehicle.lastEntry();
	}

	/**
	 * Gets odometry pose from history. Linearly interpolates between gaps.
	 *
	 * @param timestamp Timestamp to loop up.
	 * @return Odometry relative robot pose at timestamp.
	 */
	public synchronized Pose2d getOdomToVehicle(double timestamp) {
		return odometry_to_vehicle.getInterpolated(new InterpolatingDouble(timestamp));
	}

	/**
	 * Gets interpolated odometry pose using predicted robot velocity from latest
	 * odometry update.
	 *
	 * @param lookahead_time Scalar for predicted velocity.
	 * @return Predcited odometry pose at lookahead time.
	 */
	public synchronized Pose2d getPredictedOdomToVehicle(double lookahead_time) {
		return getLatestOdomToVehicle()
				.getValue()
				.transformBy(Pose2d.exp(vehicle_velocity_predicted.scaled(lookahead_time)));
	}

	/**
	 * @return Latest odometry error translation.
	 */
	public synchronized Translation2d getLatestFieldToOdom() {
		return getFieldToOdom(field_to_odometry.lastKey().value);
	}

	/**
	 * Gets odometry error translation at timestamp. Linearly interpolates between gaps.
	 * @param timestamp Timestamp to look up.
	 * @return Odometry error at timestamp.
	 */
	public synchronized Translation2d getFieldToOdom(double timestamp) {
		if (field_to_odometry.isEmpty()) return Translation2d.identity();
		return field_to_odometry.getInterpolated(new InterpolatingDouble(timestamp));
	}

	/**
	 * @return Predicted robot velocity from last odometry update.
	 */
	public synchronized Twist2d getPredictedVelocity() {
		return vehicle_velocity_predicted;
	}

	/**
	 * @return Measured robot velocity from last odometry update.
	 */
	public synchronized Twist2d getMeasuredVelocity() {
		return vehicle_velocity_measured;
	}

	/**
	 * @return Measured robot velocity smoothed using a moving average filter.
	 */
	public synchronized Twist2d getSmoothedVelocity() {
		return vehicle_velocity_measured_filtered.getAverage();
	}

	/**
	 * @return Gets if estimator has recieved a vision update.
	 */
	public synchronized boolean getHasRecievedVisionUpdate() {
		return mHasRecievedVisionUpdate;
	}

	/**
	 * Updates tracker to use stricter auto vision filtering.
	 * @param in_auto If auto filters should be used.
	 */
	public synchronized void setIsInAuto(boolean in_auto) {
		mIsInAuto = in_auto;
	}

	/**
	 * Class to hold information about a vision update.
	 */
	public static class VisionUpdate {
		private double timestamp;
		private Translation2d field_to_camera;
		private Translation2d robot_to_camera;
		private double xy_stdev;

		public VisionUpdate(
				double timestamp, Translation2d field_to_camera, Translation2d robot_to_camera, double xy_stdev) {
			this.timestamp = timestamp;
			this.field_to_camera = field_to_camera;
			this.robot_to_camera = robot_to_camera;
			this.xy_stdev = xy_stdev;
		}

		public double getTimestamp() {
			return timestamp;
		}

		public Translation2d getFieldToVehicle() {
			return field_to_camera;
		}

		public Translation2d getRobotToCamera() {
			return robot_to_camera;
		}

		public double getXYStdev() {
			return xy_stdev;
		}
	}
}
