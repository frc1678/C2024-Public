package com.team1678.lib.swerve;

import com.team1678.frc2024.Constants.SwerveConstants;
import com.team254.lib.util.SynchronousPIDF;
import edu.wpi.first.wpilibj.Timer;

public class SwerveHeadingController {
	private static SwerveHeadingController mInstance;

	public static SwerveHeadingController getInstance() {
		if (mInstance == null) {
			mInstance = new SwerveHeadingController();
		}

		return mInstance;
	}

	public double targetHeadingRadians;
	private double lastUpdatedTimestamp;

	public enum State {
		OFF,
		SNAP,
		STABILIZE
	}

	private State current_state = State.OFF;

	public State getState() {
		return current_state;
	}

	public void setState(State state) {
		current_state = state;
	}

	public void disable() {
		setState(State.OFF);
	}

	private SynchronousPIDF stabilizePID;
	private SynchronousPIDF snapPID;

	public void setSnapTarget(double angle_rad) {
		targetHeadingRadians = angle_rad;
		setState(State.SNAP);
	}

	public void setStabilizeTarget(double angle_rad) {
		targetHeadingRadians = angle_rad;
		setState(State.STABILIZE);
	}

	public double getTargetHeadingRadians() {
		return targetHeadingRadians;
	}

	public SwerveHeadingController() {
		stabilizePID = new SynchronousPIDF(
				SwerveConstants.kStabilizeSwerveHeadingKp,
				SwerveConstants.kStabilizeSwerveHeadingKi,
				SwerveConstants.kStabilizeSwerveHeadingKd,
				SwerveConstants.kStabilizeSwerveHeadingKf);

		snapPID = new SynchronousPIDF(
				SwerveConstants.kSnapSwerveHeadingKp,
				SwerveConstants.kSnapSwerveHeadingKi,
				SwerveConstants.kSnapSwerveHeadingKd,
				SwerveConstants.kSnapSwerveHeadingKf);

		stabilizePID.setInputRange(-Math.PI, Math.PI);
		stabilizePID.setContinuous();

		stabilizePID.setOutputRange(-10 * Math.PI, 10 * Math.PI);

		targetHeadingRadians = 0.0;
		lastUpdatedTimestamp = Timer.getFPGATimestamp();
	}

	public double update(double headingRadians, double timestamp) {
		double correction = 0;
		double error = headingRadians - targetHeadingRadians;
		double dt = timestamp - lastUpdatedTimestamp;
		switch (current_state) {
			case OFF:
				break;
			case STABILIZE:
				correction = stabilizePID.calculate(error, dt);
				break;
			case SNAP:
				correction = snapPID.calculate(error, dt);
				break;
		}

		lastUpdatedTimestamp = timestamp;
		return correction;
	}
}
