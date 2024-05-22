package com.team1678.frc2024.subsystems.servo;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team1678.frc2024.loops.ILooper;
import com.team1678.frc2024.loops.Loop;
import com.team1678.frc2024.subsystems.Subsystem;
import com.team254.lib.drivers.CanDeviceId;
import com.team254.lib.drivers.Phoenix6Util;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.drivers.TalonUtil;
import com.team254.lib.motion.MotionState;
import com.team254.lib.util.ReflectingCSVWriter;
import com.team254.lib.util.Util;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.UnaryOperator;

/**
 * Abstract base class for a subsystem with a single sensored servo-mechanism.
 * spotless:off
 */
public abstract class ServoMotorSubsystem extends Subsystem {
	protected static final int kMotionMagicSlot = 0;
	protected static final int kPositionPIDSlot = 1;

	// Recommend initializing in a static block!
	public static class TalonFXConstants {
		public CanDeviceId id = new CanDeviceId(-1);
		public boolean counterClockwisePositive = true;
		public boolean invert_sensor_phase = false;
		public int encoder_ppr = 2048;
	}

	// Recommend initializing in a static block!
	public static class ServoMotorSubsystemConstants {
		public String kName = "ERROR_ASSIGN_A_NAME";

		public double kLooperDt = 0.01;
		public double kCANTimeout = 0.010; // use for important on the fly updates
		public int kLongCANTimeoutMs = 100; // use for constructors

		public TalonFXConstants kMainConstants = new TalonFXConstants();
		public TalonFXConstants[] kFollowerConstants = new TalonFXConstants[0];

		public NeutralModeValue kNeutralMode = NeutralModeValue.Brake;
		public double kHomePosition = 0.0; // Units
		public double kRotationsPerUnitDistance = 1.0;
		public double kSoftLimitDeadband = 0.0;
		public double kKp = 0; // Raw output / raw error
		public double kKi = 0; // Raw output / sum of raw error
		public double kKd = 0; // Raw output / (err - prevErr)
		public double kKv = 0;
		public double kKa = 0;
		public double kKs = 0;
		public double kKg = 0;
		public int kDeadband = 0; // rotation

		public double kPositionKp = 0;
		public double kPositionKi = 0;
		public double kPositionKd = 0;
		public double kPositionKf = 0;
		public int kPositionDeadband = 0; // Ticks

		public double kVelocityFeedforward = 0;
		public double kArbitraryFeedforward = 0;
		public double kCruiseVelocity = 0; // Units/s
		public double kAcceleration = 0; // Units/s^2
		public double kJerk = 0; // Units/s^3
		public double kRampRate = 0.0; // s

		public int kSupplyCurrentLimit = 60; // amps
		public int kSupplyCurrentThreshold = 60;
		public double kSupplyCurrentTimeout = 0.0; // Seconds
		public boolean kEnableSupplyCurrentLimit = false;

		public int kStatorCurrentLimit = 40; // amps
		public boolean kEnableStatorCurrentLimit = false;

		public double kMaxForwardOutput = 12.0; // Volts
		public double kMaxReverseOutput = -12.0; // Volts

		public double kMaxUnitsLimit = Double.POSITIVE_INFINITY;
		public double kMinUnitsLimit = Double.NEGATIVE_INFINITY;

		public int kStatusFrame8UpdateRate = 1000;
	}

	protected final ServoMotorSubsystemConstants mConstants;
	protected final TalonFX mMain;
	protected final TalonFX[] mFollowers;

	protected TalonFXConfiguration mMainConfig;
	protected final TalonFXConfiguration[] mFollowerConfigs;

	protected final StatusSignal<Double> mMainPositionSignal;
	protected final StatusSignal<Double> mMainVelocitySignal;
	protected final StatusSignal<Double> mMainClosedLoopError;
	protected final StatusSignal<Double> mMainStatorCurrentSignal;
	protected final StatusSignal<Double> mMainSupplyCurrentSignal;
	protected final StatusSignal<Double> mMainOutputVoltageSignal;
	protected final StatusSignal<Double> mMainOutputPercentageSignal;
	protected final StatusSignal<Double> mMainClosedLoopOutputSignal;
	protected final StatusSignal<Double> mMainClosedLoopReferenceSignal;
	protected final StatusSignal<Double> mMainClosedLoopReferenceSlopeSignal;

	protected MotionState mMotionStateSetpoint = null;

	protected double mForwardSoftLimitRotations;
	protected double mReverseSoftLimitRotations;

	protected ServoMotorSubsystem(final ServoMotorSubsystemConstants constants) {
		mConstants = constants;
		mMain = TalonFXFactory.createDefaultTalon(mConstants.kMainConstants.id, false);
		mFollowers = new TalonFX[mConstants.kFollowerConstants.length];
		mFollowerConfigs = new TalonFXConfiguration[mConstants.kFollowerConstants.length];

		Phoenix6Util.checkErrorAndRetry(() -> mMain.getBridgeOutput().setUpdateFrequency(200, 0.05));
		Phoenix6Util.checkErrorAndRetry(() -> mMain.getFault_Hardware().setUpdateFrequency(4, 0.05));

		mMainPositionSignal = mMain.getPosition();
		mMainVelocitySignal = mMain.getVelocity();
		mMainClosedLoopError = mMain.getClosedLoopError();
		mMainStatorCurrentSignal = mMain.getStatorCurrent();
		mMainSupplyCurrentSignal = mMain.getSupplyCurrent();
		mMainOutputVoltageSignal = mMain.getMotorVoltage();
		mMainOutputPercentageSignal = mMain.getDutyCycle();
		mMainClosedLoopReferenceSignal = mMain.getClosedLoopReference();
		mMainClosedLoopOutputSignal = mMain.getClosedLoopOutput();
		mMainClosedLoopReferenceSlopeSignal = mMain.getClosedLoopReferenceSlope();

		Phoenix6Util.checkErrorAndRetry(() -> mMainPositionSignal.setUpdateFrequency(200, 0.05));
		Phoenix6Util.checkErrorAndRetry(() -> mMainVelocitySignal.setUpdateFrequency(200, 0.05));
		Phoenix6Util.checkErrorAndRetry(() -> mMainClosedLoopError.setUpdateFrequency(200, 0.05));
		Phoenix6Util.checkErrorAndRetry(() -> mMainStatorCurrentSignal.setUpdateFrequency(200, 0.05));
		Phoenix6Util.checkErrorAndRetry(() -> mMainOutputVoltageSignal.setUpdateFrequency(200, 0.05));
		Phoenix6Util.checkErrorAndRetry(() -> mMainOutputPercentageSignal.setUpdateFrequency(200, 0.05));
		Phoenix6Util.checkErrorAndRetry(() -> mMainClosedLoopReferenceSignal.setUpdateFrequency(200, 0.05));
		Phoenix6Util.checkErrorAndRetry(() -> mMainClosedLoopOutputSignal.setUpdateFrequency(200, 0.05));
		Phoenix6Util.checkErrorAndRetry(() -> mMainClosedLoopReferenceSlopeSignal.setUpdateFrequency(200, 0.05));

		mMainConfig = TalonFXFactory.getDefaultConfig();

		mMainConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
		mMainConfig.Feedback.SensorToMechanismRatio = (mConstants.kMainConstants.invert_sensor_phase ? -1 : 1);

		mForwardSoftLimitRotations =
				(((mConstants.kMaxUnitsLimit - mConstants.kHomePosition) * mConstants.kRotationsPerUnitDistance)
						- mConstants.kSoftLimitDeadband);
		mMainConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = mForwardSoftLimitRotations;
		mMainConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

		mReverseSoftLimitRotations =
				(((mConstants.kMinUnitsLimit - mConstants.kHomePosition) * mConstants.kRotationsPerUnitDistance)
						+ mConstants.kSoftLimitDeadband);
		mMainConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = mReverseSoftLimitRotations;
		mMainConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

		mMainConfig.Slot0.kP = mConstants.kKp;
		mMainConfig.Slot0.kI = mConstants.kKi;
		mMainConfig.Slot0.kD = mConstants.kKd;
		mMainConfig.Slot0.kV = mConstants.kKv;
		mMainConfig.Slot0.kG = mConstants.kKg;
		mMainConfig.Slot0.kA = mConstants.kKa;
		mMainConfig.Slot0.kS = mConstants.kKs;

		mMainConfig.Slot1.kP = mConstants.kPositionKp;
		mMainConfig.Slot1.kI = mConstants.kPositionKi;
		mMainConfig.Slot1.kD = mConstants.kPositionKd;
		mMainConfig.Slot1.kV = mConstants.kVelocityFeedforward;
		mMainConfig.MotionMagic.MotionMagicCruiseVelocity = unitsToRotations(mConstants.kCruiseVelocity);
		mMainConfig.MotionMagic.MotionMagicAcceleration = unitsToRotations(mConstants.kAcceleration);
		mMainConfig.MotionMagic.MotionMagicJerk = mConstants.kJerk;

		mMainConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = mConstants.kRampRate;
		mMainConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = mConstants.kRampRate;
		mMainConfig.OpenLoopRamps.TorqueOpenLoopRampPeriod = mConstants.kRampRate;

		mMainConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = mConstants.kRampRate;
		mMainConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = mConstants.kRampRate;
		mMainConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = mConstants.kRampRate;

		mMainConfig.CurrentLimits.SupplyCurrentLimit = mConstants.kSupplyCurrentLimit;
		mMainConfig.CurrentLimits.SupplyCurrentThreshold = mConstants.kSupplyCurrentThreshold;
		mMainConfig.CurrentLimits.SupplyTimeThreshold = mConstants.kSupplyCurrentTimeout;
		mMainConfig.CurrentLimits.SupplyCurrentLimitEnable = mConstants.kEnableSupplyCurrentLimit;

		mMainConfig.CurrentLimits.StatorCurrentLimit = mConstants.kStatorCurrentLimit;
		mMainConfig.CurrentLimits.StatorCurrentLimitEnable = mConstants.kEnableStatorCurrentLimit;

		mMainConfig.Voltage.PeakForwardVoltage = mConstants.kMaxForwardOutput;
		mMainConfig.Voltage.PeakReverseVoltage = mConstants.kMaxReverseOutput;

		mMainConfig.MotorOutput.PeakForwardDutyCycle = mConstants.kMaxForwardOutput / 12.0;
		mMainConfig.MotorOutput.PeakReverseDutyCycle = mConstants.kMaxReverseOutput / 12.0;

		mMainConfig.MotorOutput.Inverted = (mConstants.kMainConstants.counterClockwisePositive
				? InvertedValue.CounterClockwise_Positive
				: InvertedValue.Clockwise_Positive);

		mMainConfig.MotorOutput.NeutralMode = mConstants.kNeutralMode;

		for (int i = 0; i < mFollowers.length; ++i) {
			mFollowers[i] = TalonFXFactory.createPermanentFollowerTalon(
					mConstants.kFollowerConstants[i].id, mConstants.kMainConstants.id, false);

			TalonFX follower = mFollowers[i];
			mFollowerConfigs[i] = new TalonFXConfiguration();
			TalonFXConfiguration followerConfig = mFollowerConfigs[i];
			Phoenix6Util.checkErrorAndRetry(() -> follower.getConfigurator().refresh(followerConfig));

			followerConfig.MotorOutput.Inverted = (mConstants.kMainConstants.counterClockwisePositive
					? InvertedValue.CounterClockwise_Positive
					: InvertedValue.Clockwise_Positive);
			followerConfig.MotorOutput.NeutralMode = mConstants.kNeutralMode;
			followerConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
			followerConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
			follower.setControl(new Follower(mConstants.kMainConstants.id.getDeviceNumber(), false));

			TalonUtil.applyAndCheckConfiguration(follower, followerConfig);
		}

		TalonUtil.applyAndCheckConfiguration(mMain, mMainConfig);

		// Send a neutral command.
		stop();
	}

	public void setStatorCurrentLimit(double currentLimit, boolean enable) {
		changeTalonConfig((conf) -> {
			conf.CurrentLimits.StatorCurrentLimit = currentLimit;
			conf.CurrentLimits.StatorCurrentLimitEnable = enable;
			return conf;
		});
	}

	public void enableSoftLimits(boolean enable) {
		UnaryOperator<TalonFXConfiguration> configChanger = (conf) -> {
			conf.SoftwareLimitSwitch.ForwardSoftLimitEnable = enable;
			conf.SoftwareLimitSwitch.ReverseSoftLimitEnable = enable;
			return conf;
		};
		mMainConfig = configChanger.apply(mMainConfig);

		configChanger = (conf) -> {
			conf.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
			conf.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
			return conf;
		};
		for (int i = 0; i < mFollowers.length; ++i) {
			mFollowerConfigs[i] = configChanger.apply(mFollowerConfigs[i]);
		}

		writeConfigs();
	}

	public void setNeutralMode(NeutralModeValue mode) {
		changeTalonConfig((conf) -> {
			conf.MotorOutput.NeutralMode = mode;
			return conf;
		});
	}

	public void changeTalonConfig(UnaryOperator<TalonFXConfiguration> configChanger) {
		for (int i = 0; i < mFollowers.length; ++i) {
			mFollowerConfigs[i] = configChanger.apply(mFollowerConfigs[i]);
		}
		mMainConfig = configChanger.apply(mMainConfig);
		writeConfigs();
	}

	public void writeConfigs() {
		for (int i = 0; i < mFollowers.length; ++i) {
			TalonFX follower = mFollowers[i];
			TalonFXConfiguration followerConfig = mFollowerConfigs[i];
			TalonUtil.applyAndCheckConfiguration(follower, followerConfig);
		}
		TalonUtil.applyAndCheckConfiguration(mMain, mMainConfig);
	}

	public static class PeriodicIO implements Sendable {
		// INPUTS
		public double timestamp;
		public double position_rots; // motor rotations
		public double position_units;
		public double velocity_rps;
		public double prev_vel_rps;
		public double output_percent;
		public double output_voltage;
		public double main_stator_current;
		public double main_supply_current;
		public double error_rotations;
		public boolean reset_occured;
		public double active_trajectory_position;
		public double active_trajectory_velocity;
		public double active_trajectory_acceleration;

		// OUTPUTS
		public double demand; // position (motor rots) or percent output

		@Override
		public void initSendable(SendableBuilder builder) {
			builder.addDoubleProperty("PositionRots", () -> position_rots, null);
			builder.addDoubleProperty("PositionUnits", () -> position_units, null);
			builder.addDoubleProperty("VelocityRpS", () -> velocity_rps, null);
			builder.addDoubleProperty("OutputVoltage", () -> output_voltage, null);
			builder.addDoubleProperty("StatorCurrent", () -> main_stator_current, null);
			builder.addDoubleProperty("SupplyCurrent", () -> main_supply_current, null);
			builder.addDoubleProperty("ActiveTrajectoryPosition", () -> active_trajectory_position, null);
			builder.addDoubleProperty("Demand", () -> demand, null);
		}
	}

	protected enum ControlState {
		OPEN_LOOP,
		MOTION_MAGIC,
		POSITION_PID
	}

	protected PeriodicIO mPeriodicIO = new PeriodicIO();
	protected ControlState mControlState = ControlState.OPEN_LOOP;
	protected ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;
	protected boolean mHasBeenZeroed = false;
	protected StatusSignal<Integer> mMainStickyFault;

	@Override
	public synchronized void readPeriodicInputs() {
		mPeriodicIO.timestamp = Timer.getFPGATimestamp();

		if (mMain.hasResetOccurred()) {
			DriverStation.reportError(mConstants.kName + ": Talon Reset! ", false);
			mPeriodicIO.reset_occured = true;
			return;
		} else {
			mPeriodicIO.reset_occured = false;
		}

		mMainStickyFault = mMain.getStickyFaultField();

		if (mMain.getControlMode().getValue() == ControlModeValue.PositionDutyCycle) {
			mPeriodicIO.error_rotations = mMainClosedLoopError.asSupplier().get();
		} else {
			mPeriodicIO.error_rotations = 0;
		}
		mPeriodicIO.error_rotations = mMainClosedLoopError.asSupplier().get();
		mPeriodicIO.main_stator_current = mMainStatorCurrentSignal.asSupplier().get();
		mPeriodicIO.main_supply_current = mMainSupplyCurrentSignal.asSupplier().get();
		mPeriodicIO.output_voltage = mMainOutputVoltageSignal.asSupplier().get();
		mPeriodicIO.output_percent = mMainOutputPercentageSignal.asSupplier().get();
		mPeriodicIO.position_rots = mMainPositionSignal.asSupplier().get();
		mPeriodicIO.position_units = rotationsToHomedUnits(mPeriodicIO.position_rots);
		mPeriodicIO.velocity_rps = mMainVelocitySignal.asSupplier().get();
		mPeriodicIO.active_trajectory_position =
				mMainClosedLoopReferenceSignal.asSupplier().get();

		final double newVelocity =
				mMainClosedLoopReferenceSlopeSignal.asSupplier().get();
		if (Util.epsilonEquals(newVelocity, mConstants.kCruiseVelocity, Math.max(1, mConstants.kDeadband))
				|| Util.epsilonEquals(
						newVelocity, mPeriodicIO.active_trajectory_velocity, Math.max(1, mConstants.kDeadband))) {
			// Mechanism is ~constant velocity.
			mPeriodicIO.active_trajectory_acceleration = 0.0;
		} else {
			// Mechanism is accelerating.
			mPeriodicIO.active_trajectory_acceleration =
					Math.signum(newVelocity - mPeriodicIO.active_trajectory_velocity) * mConstants.kAcceleration;
		}
		mPeriodicIO.active_trajectory_velocity = newVelocity;

		if (mCSVWriter != null) {
			mCSVWriter.add(mPeriodicIO);
		}
	}

	@Override
	public synchronized void writePeriodicOutputs() {
		if (mControlState == ControlState.MOTION_MAGIC) {
			mMain.setControl(new MotionMagicVoltage(mPeriodicIO.demand).withSlot(kMotionMagicSlot));
		} else if (mControlState == ControlState.POSITION_PID) {
			mMain.setControl(new PositionDutyCycle(mPeriodicIO.demand).withSlot(kPositionPIDSlot));
		} else {
			mMain.setControl(new DutyCycleOut(mPeriodicIO.demand));
		}
	}

	public synchronized void handleMainReset(boolean reset) {}

	@Override
	public void registerEnabledLoops(ILooper mEnabledLooper) {
		mEnabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {}

			@Override
			public void onLoop(double timestamp) {
				if (mPeriodicIO.reset_occured) {
					System.out.println(mConstants.kName + ": Main Talon reset occurred; resetting frame rates.");
					Phoenix6Util.checkErrorAndRetry(() -> mMainPositionSignal.setUpdateFrequency(200, 0.05));
					Phoenix6Util.checkErrorAndRetry(() -> mMainVelocitySignal.setUpdateFrequency(200, 0.05));
					Phoenix6Util.checkErrorAndRetry(() -> mMainClosedLoopError.setUpdateFrequency(200, 0.05));
					Phoenix6Util.checkErrorAndRetry(() -> mMainStatorCurrentSignal.setUpdateFrequency(200, 0.05));
					Phoenix6Util.checkErrorAndRetry(() -> mMainOutputVoltageSignal.setUpdateFrequency(200, 0.05));
					Phoenix6Util.checkErrorAndRetry(() -> mMainOutputPercentageSignal.setUpdateFrequency(200, 0.05));

					resetIfAtHome();
				}
				handleMainReset(mPeriodicIO.reset_occured);
				for (TalonFX follower : mFollowers) {
					if (follower.hasResetOccurred()) {
						System.out.println(mConstants.kName + ": Follower Talon reset occurred");
					}
				}
			}

			@Override
			public void onStop(double timestamp) {
				if (mCSVWriter != null) {
					mCSVWriter.flush();
					mCSVWriter = null;
				}

				stop();
			}
		});
	}

	public synchronized double getPositionRotations() {
		return mPeriodicIO.position_rots;
	}

	// In "Units"
	public synchronized double getPosition() {
		return rotationsToHomedUnits(mPeriodicIO.position_rots);
	}

	// In "Units per second"
	public synchronized double getVelocity() {
		return rotationsToUnits(mPeriodicIO.velocity_rps);
	}

	public synchronized double getVelError() {
		if (mMotionStateSetpoint == null) {
			return 0.0;
		}
		return rotationsToUnits(mMotionStateSetpoint.vel() - mPeriodicIO.velocity_rps);
	}

	public synchronized boolean hasFinishedTrajectory() {
		return Util.epsilonEquals(
				mPeriodicIO.active_trajectory_position, getSetpoint(), Math.max(1, mConstants.kDeadband));
	}

	public synchronized double getSetpoint() {
		return (mControlState == ControlState.MOTION_MAGIC || mControlState == ControlState.POSITION_PID)
				? rotationsToHomedUnits(mPeriodicIO.demand)
				: Double.NaN;
	}

	public synchronized double getSetpointHomed() {
		return (mControlState == ControlState.MOTION_MAGIC || mControlState == ControlState.POSITION_PID)
				? rotationsToHomedUnits(mPeriodicIO.demand)
				: Double.NaN;
	}

	public synchronized void setSetpointMotionMagic(double units, double feedforward_v) {
		mPeriodicIO.demand = constrainRotations(homeAwareUnitsToRotations(units));
		if (mControlState != ControlState.MOTION_MAGIC) {
			mControlState = ControlState.MOTION_MAGIC;
		}
	}

	public synchronized void setSetpointMotionMagic(double units) {
		setSetpointMotionMagic(units, 0.0);
	}

	public synchronized void setSetpointPositionPID(double units, double feedforward_v) {
		mPeriodicIO.demand = constrainRotations(homeAwareUnitsToRotations(units));
		if (mControlState != ControlState.POSITION_PID) {
			mControlState = ControlState.POSITION_PID;
		}
	}

	public synchronized void setSetpointPositionPID(double units) {
		setSetpointPositionPID(units, 0.0);
	}

	protected double rotationsToUnits(double rotations) {
		return rotations / mConstants.kRotationsPerUnitDistance;
	}

	protected double rotationsToHomedUnits(double rotations) {
		double val = rotationsToUnits(rotations);
		return val + mConstants.kHomePosition;
	}

	protected double unitsToRotations(double units) {
		return units * mConstants.kRotationsPerUnitDistance;
	}

	protected double homeAwareUnitsToRotations(double units) {
		return unitsToRotations(units - mConstants.kHomePosition);
	}

	protected double constrainRotations(double rotations) {
		return Util.limit(rotations, mReverseSoftLimitRotations, mForwardSoftLimitRotations);
	}

	public synchronized void setOpenLoop(double percentage) {
		if (mControlState != ControlState.OPEN_LOOP) {
			mControlState = ControlState.OPEN_LOOP;
		}
		mPeriodicIO.demand = percentage;
	}

	public synchronized double getActiveTrajectoryPosition() {
		return rotationsToHomedUnits((mPeriodicIO.active_trajectory_position));
	}

	public synchronized String getControlState() {
		return mControlState.toString();
	}

	public synchronized double getPredictedPositionUnits(double lookahead_secs) {
		double predicted_units = mPeriodicIO.active_trajectory_position
				+ lookahead_secs * mPeriodicIO.active_trajectory_velocity
				+ 0.5 * mPeriodicIO.active_trajectory_acceleration * lookahead_secs * lookahead_secs;
		if (mPeriodicIO.demand >= mPeriodicIO.active_trajectory_position) {
			return Math.min(predicted_units, mPeriodicIO.demand);
		} else {
			return Math.max(predicted_units, mPeriodicIO.demand);
		}
	}

	public boolean atHomingLocation() {
		return false;
	}

	public synchronized void resetIfAtHome() {
		if (atHomingLocation()) {
			zeroSensors();
		}
	}

	@Override
	public synchronized void zeroSensors() {
		Phoenix6Util.checkErrorAndRetry(() -> mMain.setPosition(0, mConstants.kCANTimeout));
		mHasBeenZeroed = true;
	}

	public synchronized void forceZero() {
		Phoenix6Util.checkErrorAndRetry(() -> mMain.setPosition(0, mConstants.kCANTimeout));
	}

	public synchronized boolean hasBeenZeroed() {
		return mHasBeenZeroed;
	}

	public synchronized void setSupplyCurrentLimit(double value, boolean enable) {
		mMainConfig.CurrentLimits.SupplyCurrentLimit = value;
		mMainConfig.CurrentLimits.SupplyCurrentLimitEnable = enable;

		TalonUtil.applyAndCheckConfiguration(mMain, mMainConfig);
	}

	public synchronized void setSupplyCurrentLimitUnchecked(double value, boolean enable) {
		mMainConfig.CurrentLimits.SupplyCurrentLimit = value;
		mMainConfig.CurrentLimits.SupplyCurrentLimitEnable = enable;

		mMain.getConfigurator().apply(mMainConfig);
	}

	public synchronized void setStatorCurrentLimitUnchecked(double value, boolean enable) {
		mMainConfig.CurrentLimits.StatorCurrentLimit = value;
		mMainConfig.CurrentLimits.StatorCurrentLimitEnable = enable;

		mMain.getConfigurator().apply(mMainConfig);
	}

	public synchronized void setMotionMagicConfigsUnchecked(double accel, double jerk) {
		mMainConfig.MotionMagic.MotionMagicAcceleration = accel;
		mMainConfig.MotionMagic.MotionMagicJerk = jerk;
		mMain.getConfigurator().apply(mMainConfig.MotionMagic);
	}

	public synchronized void setMotionMagicConfigs(double accel, double velocity) {
		mMainConfig.MotionMagic.MotionMagicAcceleration = unitsToRotations(accel);
		mMainConfig.MotionMagic.MotionMagicCruiseVelocity = unitsToRotations(velocity);

		TalonUtil.applyAndCheckConfiguration(mMain, mMainConfig);
	}

	@Override
	public void stop() {
		setOpenLoop(0.0);
		mMain.stopMotor();
	}

	@Override
	public synchronized void outputTelemetry() {
		SmartDashboard.putNumber(mConstants.kName + "/position units", mPeriodicIO.position_units);
		SmartDashboard.putNumber(mConstants.kName + "/position rots", mPeriodicIO.position_rots);
		SmartDashboard.putData(mConstants.kName + "/IO", mPeriodicIO);
	}

	@Override
	public void rewriteDeviceConfiguration() {
		writeConfigs();
	}

	@Override
	public boolean checkDeviceConfiguration() {
		if (!TalonUtil.readAndVerifyConfiguration(mMain, mMainConfig)) {
			return false;
		}
		for (int i = 0; i < mFollowers.length; ++i) {
			TalonFX follower = mFollowers[i];
			TalonFXConfiguration followerConfig = mFollowerConfigs[i];
			if (!TalonUtil.readAndVerifyConfiguration(follower, followerConfig)) {
				return false;
			}
		}
		return true;
	}
}
