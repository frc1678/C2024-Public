package com.team1678.lib.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team1678.frc2024.Constants;
import com.team1678.frc2024.Constants.SwerveConstants;
import com.team1678.frc2024.subsystems.Subsystem;
import com.team1678.lib.Conversions;
import com.team1678.lib.Util;
import com.team254.lib.drivers.Phoenix6Util;
import com.team254.lib.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule extends Subsystem {

	private final int kModuleNumber;
	private final double kAngleOffset;

	private TalonFX mAngleMotor;
	private TalonFX mDriveMotor;
	private CANcoder angleEncoder;

	private BaseStatusSignal[] mSignals = new BaseStatusSignal[4];

	private mPeriodicIO mPeriodicIO = new mPeriodicIO();

	public static class mPeriodicIO {
		// Inputs
		public double timestamp = 0.0;
		public double rotationPosition = 0.0;
		public double rotationVelocity = 0.0;
		public double drivePosition = 0.0;
		public double driveVelocity = 0.0;

		// Outputs
		public double targetVelocity = 0.0;
		public ControlRequest rotationDemand;
		public ControlRequest driveDemand;
	}

	public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants, CANcoder cancoder) {
		this.kModuleNumber = moduleNumber;
		kAngleOffset = moduleConstants.angleOffset;

		angleEncoder = cancoder;

		// Angle motor config
		mAngleMotor = new TalonFX(moduleConstants.angleMotorID, "canivore1");
		Phoenix6Util.checkErrorAndRetry(() ->
				mAngleMotor.getConfigurator().apply(SwerveConstants.AzimuthFXConfig(), Constants.kLongCANTimeoutMs));
		mAngleMotor.setInverted(SwerveConstants.angleMotorInvert);

		// Drive motor config
		mDriveMotor = new TalonFX(moduleConstants.driveMotorID, "canivore1");
		Phoenix6Util.checkErrorAndRetry(() ->
				mDriveMotor.getConfigurator().apply(SwerveConstants.DriveFXConfig(), Constants.kLongCANTimeoutMs));
		mDriveMotor.setInverted(SwerveConstants.driveMotorInvert);
		mDriveMotor.setPosition(0.0);

		resetToAbsolute();

		mSignals[0] = mDriveMotor.getRotorPosition();
		mSignals[1] = mDriveMotor.getRotorVelocity();
		mSignals[2] = mAngleMotor.getRotorPosition();
		mSignals[3] = mAngleMotor.getRotorVelocity();
	}

	@Override
	public synchronized void readPeriodicInputs() {
		mPeriodicIO.timestamp = Timer.getFPGATimestamp();
		refreshSignals();
	}

	public synchronized void refreshSignals() {
		mPeriodicIO.rotationVelocity = mAngleMotor.getRotorVelocity().getValue();
		mPeriodicIO.driveVelocity = mDriveMotor.getRotorVelocity().getValue();

		mPeriodicIO.rotationPosition = BaseStatusSignal.getLatencyCompensatedValue(
				mAngleMotor.getRotorPosition(), mAngleMotor.getRotorVelocity());
		mPeriodicIO.drivePosition = mDriveMotor.getRotorPosition().getValueAsDouble();
	}

	public void setOpenLoop(SwerveModuleState desiredState) {
		double flip = setSteeringAngleOptimized(new Rotation2d(desiredState.angle)) ? -1 : 1;
		mPeriodicIO.targetVelocity = desiredState.speedMetersPerSecond * flip;
		double rotorSpeed = Conversions.MPSToRPS(
				mPeriodicIO.targetVelocity, SwerveConstants.wheelCircumference, SwerveConstants.driveGearRatio);
		mPeriodicIO.driveDemand = new VoltageOut(rotorSpeed * SwerveConstants.kV)
				.withEnableFOC(true)
				.withOverrideBrakeDurNeutral(false);
	}

	public void setVelocity(SwerveModuleState desiredState) {
		double flip = setSteeringAngleOptimized(new Rotation2d(desiredState.angle)) ? -1 : 1;
		mPeriodicIO.targetVelocity = desiredState.speedMetersPerSecond * flip;
		double rotorSpeed = Conversions.MPSToRPS(
				mPeriodicIO.targetVelocity,
				Constants.SwerveConstants.wheelCircumference,
				Constants.SwerveConstants.driveGearRatio);

		if (Math.abs(rotorSpeed) < 0.002) {
			mPeriodicIO.driveDemand = new NeutralOut();
		} else {
			mPeriodicIO.driveDemand = new VelocityVoltage(rotorSpeed);
		}
	}

	private boolean setSteeringAngleOptimized(Rotation2d steerAngle) {
		boolean flip = false;
		final double targetClamped = steerAngle.getDegrees();
		final double angleUnclamped = getCurrentUnboundedDegrees();
		final Rotation2d angleClamped = Rotation2d.fromDegrees(angleUnclamped);
		final Rotation2d relativeAngle = Rotation2d.fromDegrees(targetClamped).rotateBy(angleClamped.inverse());
		double relativeDegrees = relativeAngle.getDegrees();
		if (relativeDegrees > 90.0) {
			relativeDegrees -= 180.0;
			flip = true;

		} else if (relativeDegrees < -90.0) {
			relativeDegrees += 180.0;
			flip = true;
		}
		setSteeringAngleRaw(angleUnclamped + relativeDegrees);
		target_angle = angleUnclamped + relativeDegrees;
		return flip;
	}

	private double target_angle;

	private void setSteeringAngleRaw(double angleDegrees) {
		double rotorPosition = Conversions.degreesToRotation(angleDegrees, SwerveConstants.angleGearRatio);
		mPeriodicIO.rotationDemand = new PositionDutyCycle(rotorPosition, 0.0, true, 0.0, 0, false, false, false);
	}

	@Override
	public synchronized void writePeriodicOutputs() {
		mAngleMotor.setControl(mPeriodicIO.rotationDemand);
		mDriveMotor.setControl(mPeriodicIO.driveDemand);
	}

	public void resetToAbsolute() {
		angleEncoder.getAbsolutePosition().waitForUpdate(Constants.kLongCANTimeoutMs);
		double angle = Util.placeInAppropriate0To360Scope(
				getCurrentUnboundedDegrees(), getCanCoder().getDegrees() - kAngleOffset);
		double absolutePosition = Conversions.degreesToRotation(angle, SwerveConstants.angleGearRatio);
		Phoenix6Util.checkErrorAndRetry(() -> mAngleMotor.setPosition(absolutePosition, Constants.kLongCANTimeoutMs));
	}

	public void setDriveNeutralBrake(boolean wantBrake) {
		TalonFXConfiguration t = new TalonFXConfiguration();
		mDriveMotor.getConfigurator().refresh(t);
		t.MotorOutput.NeutralMode = wantBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
		mDriveMotor.getConfigurator().apply(t);
		mAngleMotor.getConfigurator().refresh(t);
		t.MotorOutput.NeutralMode = !wantBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
		mAngleMotor.getConfigurator().apply(t);
	}

	@Override
	public void outputTelemetry() {
		// spotless:off
		SmartDashboard.putNumber("Module" + kModuleNumber + "/Azi Target", target_angle);
		SmartDashboard.putNumber("Module" + kModuleNumber + "/Azi Angle", getCurrentUnboundedDegrees());
		SmartDashboard.putNumber("Module" + kModuleNumber + "/Azi Error", getCurrentUnboundedDegrees() - target_angle);
		SmartDashboard.putNumber("Module" + kModuleNumber + "/Wheel Velocity", Math.abs(getCurrentVelocity()));
		SmartDashboard.putNumber("Module" + kModuleNumber + "/Wheel Target Velocity", Math.abs(mPeriodicIO.targetVelocity));
		SmartDashboard.putNumber("Module" + kModuleNumber + "/Drive Position", Math.abs(mPeriodicIO.drivePosition));
		SmartDashboard.putNumber("Module" + kModuleNumber + "/Duty Cycle",
				mDriveMotor.getDutyCycle().getValueAsDouble());
		SmartDashboard.putNumber("Module" + kModuleNumber + "/Azi Current",
				mAngleMotor.getSupplyCurrent().getValueAsDouble());
		SmartDashboard.putNumber("Module" + kModuleNumber + "/Drive Current",
				mDriveMotor.getSupplyCurrent().getValueAsDouble());
		SmartDashboard.putNumber("Module" + kModuleNumber + "/Wheel Velocity Error",
				Math.abs(getCurrentVelocity()) - Math.abs(mPeriodicIO.targetVelocity));
		// spotless:on
	}

	public int moduleNumber() {
		return kModuleNumber;
	}

	public double angleOffset() {
		return kAngleOffset;
	}

	public Rotation2d getCanCoder() {
		return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition().getValue() * 360.0);
	}

	public Rotation2d getModuleAngle() {
		return Rotation2d.fromDegrees(getCurrentUnboundedDegrees());
	}

	public SwerveModuleState getState() {
		return new SwerveModuleState(getCurrentVelocity(), getModuleAngle());
	}

	public SwerveModulePosition getPosition() {
		return new SwerveModulePosition(getDriveDistanceMeters(), getModuleAngle());
	}

	public edu.wpi.first.math.kinematics.SwerveModulePosition getWpiPosition() {
		return new edu.wpi.first.math.kinematics.SwerveModulePosition(
				getDriveDistanceMeters(),
				edu.wpi.first.math.geometry.Rotation2d.fromDegrees(
						getModuleAngle().getDegrees()));
	}

	public double getTargetVelocity() {
		return mPeriodicIO.targetVelocity;
	}

	public double getCurrentVelocity() {
		return Conversions.RPSToMPS(
				mPeriodicIO.driveVelocity,
				Constants.SwerveConstants.wheelCircumference,
				Constants.SwerveConstants.driveGearRatio);
	}

	public double getDriveDistanceMeters() {
		return Conversions.rotationsToMeters(
				mPeriodicIO.drivePosition,
				Constants.SwerveConstants.wheelCircumference,
				Constants.SwerveConstants.driveGearRatio);
	}

	public double getCurrentUnboundedDegrees() {
		return Conversions.rotationsToDegrees(mPeriodicIO.rotationPosition, SwerveConstants.angleGearRatio);
	}

	public double getTimestamp() {
		return mPeriodicIO.timestamp;
	}

	public double getDriveMotorCurrent() {
		return mDriveMotor.getStatorCurrent().getValue();
	}

	public BaseStatusSignal[] getUsedStatusSignals() {
		return mSignals;
	}

	public static class SwerveModuleConstants {
		public final int driveMotorID;
		public final int angleMotorID;
		public final double angleOffset;

		public SwerveModuleConstants(int driveMotorID, int angleMotorID, double angleOffset) {
			this.driveMotorID = driveMotorID;
			this.angleMotorID = angleMotorID;
			this.angleOffset = angleOffset;
		}
	}
}
