package com.team1678.lib.drivers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.team1678.frc2024.Constants;
import com.team1678.frc2024.Ports;
import com.team254.lib.geometry.Rotation2d;

public class Pigeon {

	private static Pigeon mInstance;

	public static Pigeon getInstance() {
		if (mInstance == null) {
			mInstance = new Pigeon(Ports.PIGEON);
		}
		return mInstance;
	}

	// Actual pigeon object
	private final Pigeon2 mGyro;

	// Configs
	private boolean inverted = Constants.SwerveConstants.invertGyro;
	private Rotation2d yawAdjustmentAngle = new Rotation2d();
	private Rotation2d rollAdjustmentAngle = new Rotation2d();
	private Rotation2d pitchAdjustmentAngle = new Rotation2d();

	private Pigeon(int port) {
		mGyro = new Pigeon2(port, "canivore1");
		mGyro.getConfigurator().apply(new Pigeon2Configuration());
	}

	public Rotation2d getYaw() {
		Rotation2d angle = getUnadjustedYaw().rotateBy(yawAdjustmentAngle.inverse());
		if (inverted) {
			return angle.inverse();
		}
		return angle;
	}

	public Rotation2d getRoll() {
		return getUnadjustedRoll().rotateBy(rollAdjustmentAngle.inverse());
	}

	public Rotation2d getPitch() {
		return getUnadjustedPitch().rotateBy(pitchAdjustmentAngle.inverse()).inverse();
	}

	/**
	 * Sets the yaw register to read the specified value.
	 *
	 * @param angleDeg New yaw in degrees
	 */
	public void setYaw(double angleDeg) {
		yawAdjustmentAngle = Rotation2d.fromDegrees(getYawStatusSignal().getValueAsDouble())
				.rotateBy(Rotation2d.fromDegrees(angleDeg).inverse());
	}

	/**
	 * Sets the roll register to read the specified value.
	 *
	 * @param angleDeg New yaw in degrees
	 */
	public void setRoll(double angleDeg) {
		rollAdjustmentAngle =
				getUnadjustedRoll().rotateBy(Rotation2d.fromDegrees(angleDeg).inverse());
	}

	/**
	 * Sets the roll register to read the specified value.
	 *
	 * @param angleDeg New yaw in degrees
	 */
	public void setPitch(double angleDeg) {
		pitchAdjustmentAngle =
				getUnadjustedPitch().rotateBy(Rotation2d.fromDegrees(angleDeg).inverse());
		System.out.println("Reset gyro to " + getPitch().getDegrees());
	}

	public Rotation2d getUnadjustedYaw() {
		return Rotation2d.fromDegrees(
				BaseStatusSignal.getLatencyCompensatedValue(getYawStatusSignal(), getRateStatusSignal()));
	}

	public Rotation2d getUnadjustedPitch() {
		return Rotation2d.fromDegrees(mGyro.getRoll().getValue());
	}

	public Rotation2d getUnadjustedRoll() {
		return Rotation2d.fromDegrees(mGyro.getPitch().getValue());
	}

	public StatusSignal<Double> getYawStatusSignal() {
		return mGyro.getYaw();
	}

	public StatusSignal<Double> getRateStatusSignal() {
		return mGyro.getAngularVelocityZDevice();
	}
}
