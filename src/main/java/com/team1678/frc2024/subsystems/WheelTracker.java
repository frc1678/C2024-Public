package com.team1678.frc2024.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.team1678.frc2024.Constants;
import com.team1678.lib.drivers.Pigeon;
import com.team1678.lib.swerve.SwerveModule;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.List;

public class WheelTracker {
	private final Pigeon mPigeon = Pigeon.getInstance();
	private final SwerveModule[] mModules;

	private WheelProperties[] wheels = new WheelProperties[4];
	private Pose2d robotPose = new Pose2d();
	private Translation2d robotVelocity = Translation2d.identity();

	private double robotHeading;

	private double mTimestamp;
	private boolean mIsEnabled = false;

	private BaseStatusSignal[] mAllSignals;

	private OdometryThread mOdometryThread;

	public WheelTracker(SwerveModule[] modules) {
		if (modules.length != 4) {
			throw new IllegalArgumentException("Odometry needs 4 modules to run");
		}

		mModules = modules;

		for (int i = 0; i < wheels.length; i++) {
			WheelProperties w = new WheelProperties();
			Translation2d robotToWheel = new Translation2d(
					Constants.SwerveConstants.swerveModuleLocations[i].x(),
					Constants.SwerveConstants.swerveModuleLocations[i].y());
			w.startingPosition = robotToWheel;
			wheels[i] = w;
		}

		resetModulePoses();

		mAllSignals = new BaseStatusSignal[(4 * 4) + 2];
		for (int i = 0; i < 4; ++i) {
			var signals = mModules[i].getUsedStatusSignals();
			mAllSignals[(i * 4) + 0] = signals[0];
			mAllSignals[(i * 4) + 1] = signals[1];
			mAllSignals[(i * 4) + 2] = signals[2];
			mAllSignals[(i * 4) + 3] = signals[3];
		}
		mAllSignals[mAllSignals.length - 2] = mPigeon.getYawStatusSignal();
		mAllSignals[mAllSignals.length - 1] = mPigeon.getRateStatusSignal();

		for (BaseStatusSignal sig : mAllSignals) {
			sig.setUpdateFrequency(250);
		}
		mOdometryThread = new OdometryThread();
		mOdometryThread.setDaemon(true);
		mOdometryThread.start();
	}

	public void start() {
		mIsEnabled = true;
	}

	public void stop() {
		mIsEnabled = false;
	}

	private class OdometryThread extends Thread {
		@Override
		public void run() {
			while (true) {
				try {
					BaseStatusSignal.waitForAll(1.0, mAllSignals);

					for (SwerveModule m : mModules) {
						m.refreshSignals(); // No downside to refreshing io reads from multiple threads
					}

					robotHeading = mPigeon.getYaw().getRadians();
					updateRobotPose(Timer.getFPGATimestamp());
				} catch (Exception e) {
					e.printStackTrace();
				}
			}
		}
	}

	private Pose2d last_velocity_sample = new Pose2d();
	private double last_sample_timestamp = 0.0;

	private void updateRobotPose(double timestamp) {
		double x = 0.0;
		double y = 0.0;
		Rotation2d heading = Rotation2d.fromRadians(robotHeading);

		double avg_delta = 0.0;
		double[] deltas = new double[4];
		for (int i = 0; i < mModules.length; i++) {
			SwerveModule m = mModules[i];
			WheelProperties w = wheels[i];
			updateWheelOdometry(m, w);
			double delta = w.estimatedRobotPose
					.getTranslation()
					.translateBy(robotPose.getTranslation().inverse())
					.norm();
			deltas[i] = delta;
			avg_delta += delta;
		}
		avg_delta /= 4;

		int min__dev_idx = 0;
		double min_dev = Double.MAX_VALUE;
		List<WheelProperties> accurateModules = new ArrayList<>();
		for (int i = 0; i < deltas.length; i++) {
			WheelProperties w = wheels[i];
			double dev = Math.abs(deltas[i] - avg_delta);
			if (dev < min_dev) {
				min_dev = dev;
				min__dev_idx = i;
			}
			if (dev <= 0.01) {
				accurateModules.add(w);
			}
		}

		if (accurateModules.isEmpty()) {
			accurateModules.add(wheels[min__dev_idx]);
		}

		int n = accurateModules.size();

		SmartDashboard.putNumber("Modules Used For Odometry", n);

		for (WheelProperties w : accurateModules) {
			x += w.estimatedRobotPose.getTranslation().x();
			y += w.estimatedRobotPose.getTranslation().y();
		}
		final Pose2d new_pose = new Pose2d(new Translation2d(x / n, y / n), heading);

		// Velocity calcs
		double sample_window = timestamp - last_sample_timestamp;
		if (sample_window > 0.02) {
			final Translation2d translation =
					(new_pose.transformBy(last_velocity_sample.inverse()).getTranslation());
			robotVelocity = translation.scale(1.0 / sample_window);
			last_sample_timestamp = timestamp;
			last_velocity_sample = new_pose;
		}

		robotPose = new_pose;
		resetModulePoses(robotPose);
	}

	private void updateWheelOdometry(SwerveModule module, WheelProperties props) {
		double currentEncDistance = module.getDriveDistanceMeters();
		double deltaEncDistance = currentEncDistance - props.previousEncDistance;
		Rotation2d wheelAngle = module.getModuleAngle().rotateBy(Rotation2d.fromRadians(robotHeading));
		Translation2d deltaPosition =
				new Translation2d(wheelAngle.cos() * deltaEncDistance, wheelAngle.sin() * deltaEncDistance);

		double xCorrectionFactor = 1.0;
		double yCorrectionFactor = 1.0;

		if (Math.signum(deltaPosition.x()) == 1.0) {
			xCorrectionFactor = (8.6 / 9.173);

		} else if (Math.signum(deltaPosition.x()) == -1.0) {
			xCorrectionFactor = (8.27 / 9.173);
		}

		if (Math.signum(deltaPosition.y()) == 1.0) {
			yCorrectionFactor = (3.638 / 4.0);

		} else if (Math.signum(deltaPosition.y()) == -1.0) {
			yCorrectionFactor = (3.660 / 4.0);
		}

		SmartDashboard.putString(
				"Correction Factors", String.valueOf(xCorrectionFactor) + ":" + String.valueOf(yCorrectionFactor));

		deltaPosition = new Translation2d(deltaPosition.x() * xCorrectionFactor, deltaPosition.y() * yCorrectionFactor);
		Translation2d updatedPosition = props.position.translateBy(deltaPosition);
		Pose2d wheelPose = new Pose2d(updatedPosition, Rotation2d.fromRadians(robotHeading));
		props.estimatedRobotPose = wheelPose.transformBy(Pose2d.fromTranslation(props.startingPosition.inverse()));

		props.position = updatedPosition;
		props.previousEncDistance = currentEncDistance;
	}

	private void resetModulePoses(Pose2d robotPose) {
		for (int i = 0; i < mModules.length; i++) {
			WheelProperties props = wheels[i];
			Translation2d modulePosition = robotPose
					.transformBy(Pose2d.fromTranslation(props.startingPosition))
					.getTranslation();
			props.position = modulePosition;
		}
	}

	private void resetModulePoses() {
		for (int i = 0; i < mModules.length; i++) {
			WheelProperties props = wheels[i];
			props.position = props.startingPosition;
		}
	}

	public void resetPose(Pose2d pose) {
		robotPose = pose;
		resetModulePoses(robotPose);
	}

	public class WheelProperties {
		private double previousEncDistance = 0;
		private Translation2d position;
		private Translation2d startingPosition;
		private Pose2d estimatedRobotPose = new Pose2d();
	}

	public synchronized Pose2d getRobotPose() {
		return robotPose;
	}

	public Translation2d getMeasuredVelocity() {
		return robotVelocity;
	}

	public double getTimestamp() {
		return mTimestamp;
	}

	public double wheel0_x() {
		return wheels[0].position.x();
	}

	public double wheel0_y() {
		return wheels[0].position.y();
	}

	public double wheel1_x() {
		return wheels[1].position.x();
	}

	public double wheel1_y() {
		return wheels[1].position.y();
	}

	public double wheel2_x() {
		return wheels[2].position.x();
	}

	public double wheel2_y() {
		return wheels[2].position.y();
	}

	public double wheel3_x() {
		return wheels[3].position.x();
	}

	public double wheel3_y() {
		return wheels[3].position.y();
	}

	public double robot_x() {
		return robotPose.getTranslation().x();
	}

	public double robot_y() {
		return robotPose.getTranslation().y();
	}
}
