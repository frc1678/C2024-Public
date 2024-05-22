package com.team1678.frc2024;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.team1678.frc2024.subsystems.Drive.KinematicLimits;
import com.team1678.frc2024.subsystems.limelight.GoalTracker;
import com.team1678.frc2024.subsystems.servo.ServoMotorSubsystem.ServoMotorSubsystemConstants;
import com.team1678.frc2024.subsystems.servo.ServoMotorSubsystem.TalonFXConstants;
import com.team1678.frc2024.subsystems.servo.ServoMotorSubsystemWithCancoder.AbsoluteEncoderConstants;
import com.team1678.frc2024.subsystems.vision.VisionDeviceConstants;
import com.team1678.lib.Conversions;
import com.team1678.lib.swerve.SwerveDriveKinematics;
import com.team1678.lib.swerve.SwerveModule.SwerveModuleConstants;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import java.util.Map;
import java.util.function.BooleanSupplier;

public class Constants {

	public static boolean isComp;
	public static boolean isEpsilon;

	public static BooleanSupplier isCompSupplier() {
		return () -> isComp;
	}

	public static final String kCompSerial = "032B4B47";
	public static final String kEpsilonSerial = "03260A21";

	// Disables extra smart dashboard outputs that slow down the robot
	public static final boolean disableExtraTelemetry = false;

	// robot loop time
	public static final double kLooperDt = 0.02;

	/* Control Board */
	public static final double kJoystickThreshold = 0.2;
	public static final int kButtonGamepadPort = 1;

	public static final double stickDeadband = 0.15;

	// Timeout constants
	public static final int kLongCANTimeoutMs = 100;
	public static final int kCANTimeoutMs = 10;

	public static final class SwerveConstants {
		public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

		/* Drivetrain Constants */
		public static final double trackWidth = Units.inchesToMeters(20.75);
		public static final double wheelBase = Units.inchesToMeters(20.75);

		public static final double wheelDiameter = Units.inchesToMeters(4.00);
		public static final double wheelCircumference = wheelDiameter * Math.PI;

		public static final double driveGearRatio = Constants.isEpsilon ? 5.82 : 5.82;
		public static final double angleGearRatio = (150.0 / 7.0);

		public static final Translation2d[] swerveModuleLocations = {
			new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
			new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
			new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
			new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
		};

		public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(swerveModuleLocations);

		public static final edu.wpi.first.math.geometry.Translation2d[] wpiModuleLocations = {
			new edu.wpi.first.math.geometry.Translation2d(wheelBase / 2.0, trackWidth / 2.0),
			new edu.wpi.first.math.geometry.Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
			new edu.wpi.first.math.geometry.Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
			new edu.wpi.first.math.geometry.Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
		};

		public static final edu.wpi.first.math.kinematics.SwerveDriveKinematics kWpiKinematics =
				new edu.wpi.first.math.kinematics.SwerveDriveKinematics(wpiModuleLocations);

		/* Swerve Profiling Values */
		public static final double maxSpeed = 5.2; // meters per second
		public static final double maxAngularVelocity = 11.5;

		public static final double kV = 12 * Math.PI * wheelDiameter / (driveGearRatio * maxSpeed);
		public static final double maxAutoSpeed = maxSpeed * 0.85; // Max out at 85% to ensure attainable speeds

		/* Motor Inverts */
		public static final boolean driveMotorInvert = false;
		public static final boolean angleMotorInvert = true;

		/* Angle Encoder Invert */
		public static final SensorDirectionValue canCoderInvert = SensorDirectionValue.CounterClockwise_Positive;

		/* Controller Invert */
		public static final boolean invertYAxis = false;
		public static final boolean invertRAxis = false;
		public static final boolean invertXAxis = false;

		/* Heading Controller */

		// Stabilize Heading PID Values
		public static final double kStabilizeSwerveHeadingKp = 10.0;
		public static final double kStabilizeSwerveHeadingKi = 0.0;
		public static final double kStabilizeSwerveHeadingKd = 0.3;
		public static final double kStabilizeSwerveHeadingKf = 2.0;

		// Snap Heading PID Values
		public static final double kSnapSwerveHeadingKp = 10.0;
		public static final double kSnapSwerveHeadingKi = 0.0;
		public static final double kSnapSwerveHeadingKd = 0.6;
		public static final double kSnapSwerveHeadingKf = 1.0;

		public static final KinematicLimits kUncappedLimits = new KinematicLimits();

		static {
			kUncappedLimits.kMaxDriveVelocity = maxSpeed;
			kUncappedLimits.kMaxAccel = Double.MAX_VALUE;
			kUncappedLimits.kMaxAngularVelocity = maxAngularVelocity;
			kUncappedLimits.kMaxAngularAccel = Double.MAX_VALUE;
		}

		/*** MODULE SPECIFIC CONSTANTS ***/
		/* Front Left Module - Module 0 */
		public static final class Mod0 {
			public static final double compAngleOffset = 99.5;
			public static final double epsilonAngleOffset = 0.87;

			public static SwerveModuleConstants SwerveModuleConstants() {
				return new SwerveModuleConstants(
						Ports.FL_DRIVE.getDeviceNumber(),
						Ports.FL_ROTATION.getDeviceNumber(),
						isEpsilon ? epsilonAngleOffset : compAngleOffset);
			}
		}

		/* Front Right Module - Module 1 */
		public static final class Mod1 {
			public static final double compAngleOffset = 254.880;
			public static final double epsilonAngleOffset = 67.68;

			public static SwerveModuleConstants SwerveModuleConstants() {
				return new SwerveModuleConstants(
						Ports.FR_DRIVE.getDeviceNumber(),
						Ports.FR_ROTATION.getDeviceNumber(),
						isEpsilon ? epsilonAngleOffset : compAngleOffset);
			}
		}

		/* Back Left Module - Module 2 */
		public static final class Mod2 {
			public static final double compAngleOffset = 180.0;
			public static final double epsilonAngleOffset = 46.8;

			public static SwerveModuleConstants SwerveModuleConstants() {
				return new SwerveModuleConstants(
						Ports.BL_DRIVE.getDeviceNumber(),
						Ports.BL_ROTATION.getDeviceNumber(),
						isEpsilon ? epsilonAngleOffset : compAngleOffset);
			}
		}

		/* Back Right Module - Module 3 */
		public static final class Mod3 {
			public static final double compAngleOffset = 181.5822;
			public static final double epsilonAngleOffset = 61.56;

			public static SwerveModuleConstants SwerveModuleConstants() {
				return new SwerveModuleConstants(
						Ports.BR_DRIVE.getDeviceNumber(),
						Ports.BR_ROTATION.getDeviceNumber(),
						isEpsilon ? epsilonAngleOffset : compAngleOffset);
			}
		}

		public static TalonFXConfiguration DriveFXConfig() {
			TalonFXConfiguration config = new TalonFXConfiguration();

			config.Slot0.kP = 0.030 * 12.0;
			config.Slot0.kI = 0.0;
			config.Slot0.kD = 0.000001 * 12.0;
			config.Slot0.kS = 0.1;
			config.Slot0.kV = 12 * Math.PI * wheelDiameter / (driveGearRatio * maxSpeed);

			config.CurrentLimits.StatorCurrentLimitEnable = true;
			config.CurrentLimits.StatorCurrentLimit = 110;

			config.CurrentLimits.SupplyCurrentLimitEnable = true;
			config.CurrentLimits.SupplyCurrentLimit = 90;
			config.CurrentLimits.SupplyTimeThreshold = 0.5;

			config.Voltage.PeakForwardVoltage = 12.0;
			config.Voltage.PeakReverseVoltage = -12.0;

			config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

			config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25;
			config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.25;
			return config;
		}

		public static TalonFXConfiguration AzimuthFXConfig() {
			TalonFXConfiguration config = new TalonFXConfiguration();

			config.Slot0.kP = 1.0005;
			config.Slot0.kI = 0.0;
			config.Slot0.kD = 0.0004;
			config.Slot0.kS = 0.0;
			config.Slot0.kV = 0.0;

			config.CurrentLimits.StatorCurrentLimitEnable = true;
			config.CurrentLimits.StatorCurrentLimit = 80;

			config.CurrentLimits.SupplyCurrentLimitEnable = true;
			config.CurrentLimits.SupplyCurrentLimit = 60;
			config.CurrentLimits.SupplyTimeThreshold = 0.2;

			config.Voltage.PeakForwardVoltage = 12.0;
			config.Voltage.PeakReverseVoltage = -12.0;

			config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

			return config;
		}

		public static CANcoderConfiguration swerveCancoderConfig() {
			CANcoderConfiguration CANCoderConfig = new com.ctre.phoenix6.configs.CANcoderConfiguration();
			CANCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
			CANCoderConfig.MagnetSensor.SensorDirection = Constants.SwerveConstants.canCoderInvert;
			return CANCoderConfig;
		}

		public static final double kCancoderBootAllowanceSeconds = 10.0;
	}

	public static final class IntakeRollerConstants {
		public static TalonFXConfiguration RollerFXConfig() {
			TalonFXConfiguration config = new TalonFXConfiguration();

			config.CurrentLimits.SupplyCurrentLimitEnable = true;
			config.CurrentLimits.SupplyCurrentLimit = 40.0;

			config.CurrentLimits.StatorCurrentLimitEnable = true;
			config.CurrentLimits.StatorCurrentLimit = 80.0;

			config.Voltage.PeakForwardVoltage = 12.0;
			config.Voltage.PeakReverseVoltage = -12.0;

			config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
			return config;
		}
	}

	public static final class IntakeDeployConstants {
		// 115.93
		// 7.92
		public static final ServoMotorSubsystemConstants kDeployServoConstants = new ServoMotorSubsystemConstants();

		static {
			kDeployServoConstants.kName = "Deploy";

			kDeployServoConstants.kMainConstants.id = Ports.INTAKE_PIVOT;
			kDeployServoConstants.kMainConstants.counterClockwisePositive = false;

			kDeployServoConstants.kHomePosition = 128.1; // degrees
			kDeployServoConstants.kRotationsPerUnitDistance = (1.0 / 360.0) * (45.0 / 1.0);

			kDeployServoConstants.kMaxUnitsLimit = 128.1;
			kDeployServoConstants.kMinUnitsLimit = 0.0;

			kDeployServoConstants.kKp = 3.0;
			kDeployServoConstants.kKi = 0.0;
			kDeployServoConstants.kKd = 0.0;
			kDeployServoConstants.kKa = 0.0;
			kDeployServoConstants.kKs = 0.2;
			kDeployServoConstants.kKg = 0.2;

			kDeployServoConstants.kCruiseVelocity = 400.0; // degrees / s
			kDeployServoConstants.kAcceleration = 10000.0; // degrees / s^2

			kDeployServoConstants.kMaxForwardOutput = 12.0;
			kDeployServoConstants.kMaxReverseOutput = -12.0;

			kDeployServoConstants.kEnableSupplyCurrentLimit = true;
			kDeployServoConstants.kSupplyCurrentLimit = 40; // amps
			kDeployServoConstants.kSupplyCurrentThreshold = 40; // amps
			kDeployServoConstants.kSupplyCurrentTimeout = 0.01; // seconds

			kDeployServoConstants.kEnableStatorCurrentLimit = true;
			kDeployServoConstants.kStatorCurrentLimit = 80; // amps

			kDeployServoConstants.kNeutralMode = NeutralModeValue.Brake;
		}

		public static double kHomingZone = 7.0; // degrees
		public static double kHomingTimeout = 0.2; // seconds
		public static double kHomingVelocityWindow = 5.0; // "units" / second
		public static double kHomingOutput = 4.0; // volts
	}

	public static final class SerializerConstants {
		public static TalonFXConfiguration SeralizerFXConfig() {
			TalonFXConfiguration config = new TalonFXConfiguration();

			config.CurrentLimits.StatorCurrentLimitEnable = true;
			config.CurrentLimits.StatorCurrentLimit = 80;

			config.CurrentLimits.SupplyCurrentLimit = 40.0;
			config.CurrentLimits.SupplyTimeThreshold = 0.1;
			config.CurrentLimits.SupplyCurrentLimitEnable = true;

			config.Voltage.PeakForwardVoltage = 12.0;
			config.Voltage.PeakReverseVoltage = -12.0;

			config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
			return config;
		}
	}

	public static final class AmpRollerConstants {
		public static TalonFXConfiguration AmpRollerFXConfig() {
			TalonFXConfiguration config = new TalonFXConfiguration();

			config.CurrentLimits.StatorCurrentLimitEnable = true;
			config.CurrentLimits.StatorCurrentLimit = 80;

			config.Voltage.PeakForwardVoltage = 12.0;
			config.Voltage.PeakReverseVoltage = -12.0;

			config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
			return config;
		}
	}

	public static final class FeederConstants {
		public static double kFeederVelocityConversion = (60.0) * (25.0 / 24.0); // Wheel RPMs per Rotor RPS

		public static TalonFXConfiguration FeederFXConfig() {
			TalonFXConfiguration config = new TalonFXConfiguration();

			config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

			config.Slot0.kP = 0.0;
			config.Slot0.kI = 0.0;
			config.Slot0.kD = 0.0;
			config.Slot0.kV = 12.0 / 100.0;
			config.Slot0.kS = 0.2;

			config.CurrentLimits.StatorCurrentLimitEnable = false;
			config.CurrentLimits.StatorCurrentLimit = 80;

			config.Voltage.PeakForwardVoltage = 12.0;
			config.Voltage.PeakReverseVoltage = -12.0;

			config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

			return config;
		}
	}

	public static final class ElevatorConstants {
		public static final ServoMotorSubsystemConstants kElevatorServoConstants = new ServoMotorSubsystemConstants();

		static {
			kElevatorServoConstants.kName = "Elevator";

			kElevatorServoConstants.kMainConstants.counterClockwisePositive = false;
			kElevatorServoConstants.kMainConstants.invert_sensor_phase = false;

			kElevatorServoConstants.kFollowerConstants = new TalonFXConstants[1];
			kElevatorServoConstants.kFollowerConstants[0] = new TalonFXConstants();

			kElevatorServoConstants.kMainConstants.id = Ports.ELEVATOR_MAIN;
			kElevatorServoConstants.kFollowerConstants[0].id = Ports.ELEVATOR_FOLLOWER;

			kElevatorServoConstants.kHomePosition = 0.0; // meters

			kElevatorServoConstants.kMaxUnitsLimit = 0.46;
			kElevatorServoConstants.kMinUnitsLimit = 0.0;

			kElevatorServoConstants.kRotationsPerUnitDistance = (9.0) / (Conversions.inchesToMeters(1.432) * Math.PI);

			kElevatorServoConstants.kKp = 1.0; // Raw output / raw error
			kElevatorServoConstants.kKi = 0.0; // Raw output / sum of raw error
			kElevatorServoConstants.kKd = 0.0; // Raw output / (err - prevErr)
			kElevatorServoConstants.kKa = 0.0; // Raw output / accel in (rots/s) / s
			kElevatorServoConstants.kKg = 0.2;
			kElevatorServoConstants.kDeadband = 0; // rots

			kElevatorServoConstants.kCruiseVelocity = 1.0; // m / s
			kElevatorServoConstants.kAcceleration = 120.0; // m / s^2
			kElevatorServoConstants.kRampRate = 0.0; // s

			kElevatorServoConstants.kMaxForwardOutput = 12.0;
			kElevatorServoConstants.kMaxReverseOutput = -12.0;

			kElevatorServoConstants.kEnableSupplyCurrentLimit = true;
			kElevatorServoConstants.kSupplyCurrentLimit = 40; // amps
			kElevatorServoConstants.kSupplyCurrentThreshold = 40; // amps
			kElevatorServoConstants.kSupplyCurrentTimeout = 0.01; // seconds

			kElevatorServoConstants.kNeutralMode = NeutralModeValue.Brake;
		}

		public static double kHomingZone = 0.1; // meters
		public static double kHomingTimeout = 0.5; // seconds
		public static double kHomingVelocityWindow = 0.1; // "units" / second
		public static double kHomingOutput = -2.0; // volts
	}

	public static final class ShooterConstants {
		public static final double kCompGearRatio = 1.6;
		public static final double kEpsilonTopGearRatio = 1.6;
		public static final double kEpsilonBottomGearRatio = 1.6;
		public static final double kTopFlywheelVelocityConversion =
				(60.0) * (isEpsilon ? kEpsilonTopGearRatio : kCompGearRatio) / (1.0);
		public static final double kBottomFlywheelVelocityConversion =
				(60.0) * (isEpsilon ? kEpsilonBottomGearRatio : kCompGearRatio) / (1.0);
		public static final double kFlywheelTolerance = 1000;

		public static TalonFXConfiguration ShooterFXConfig() {
			TalonFXConfiguration config = new TalonFXConfiguration();

			config.Slot0.kP = 0.0;
			config.Slot0.kI = 0.0;
			config.Slot0.kD = 0.0;
			config.Slot0.kV = 12.0 / (100.0);
			config.Slot0.kS = 0.15;

			config.CurrentLimits.SupplyCurrentLimit = 20.0;
			config.CurrentLimits.SupplyCurrentLimitEnable = true;
			config.CurrentLimits.SupplyTimeThreshold = 0.5;

			config.CurrentLimits.StatorCurrentLimitEnable = true;
			config.CurrentLimits.StatorCurrentLimit = 80;

			config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
			return config;
		}
	}

	public static final class HoodConstants {

		public static final double kRotorRotationsPerOutputRotation = 314.0 / 1.0; // Rotor to unit distance

		public static final ServoMotorSubsystemConstants kHoodServoConstants = new ServoMotorSubsystemConstants();
		public static final AbsoluteEncoderConstants kHoodEncoderConstants = new AbsoluteEncoderConstants();

		public static final double kHomingVoltage = -2.0;
		public static final double kHomingCurrentThreshold = 10.0;
		public static final double kMinHomingTime = 0.2;
		public static final double kMaxHomingTime = 4.0;

		static {
			kHoodServoConstants.kName = "Hood";

			kHoodServoConstants.kMainConstants.id = Ports.HOOD;
			kHoodServoConstants.kMainConstants.counterClockwisePositive = false;

			kHoodServoConstants.kHomePosition = 15.0; // Degrees
			kHoodServoConstants.kRotationsPerUnitDistance = (1.0 / 360.0) * (7.16 / 1.0); // Cancoder to unit distance
			kHoodServoConstants.kKp = isEpsilon ? 100 : 200;
			kHoodServoConstants.kKi = 0;
			kHoodServoConstants.kKd = 0.0;
			kHoodServoConstants.kKg = 0.7;
			kHoodServoConstants.kKs = 0.02;
			kHoodServoConstants.kDeadband = 0; // Ticks

			kHoodServoConstants.kMinUnitsLimit = 15.0;
			kHoodServoConstants.kMaxUnitsLimit = 62.0;

			kHoodServoConstants.kCruiseVelocity = 200.0; // degrees / s
			kHoodServoConstants.kAcceleration = 2400.0; // degrees / s^2

			kHoodServoConstants.kEnableSupplyCurrentLimit = true;
			kHoodServoConstants.kSupplyCurrentLimit = 40;
			kHoodServoConstants.kSupplyCurrentThreshold = 40;

			kHoodServoConstants.kEnableStatorCurrentLimit = true;
			kHoodServoConstants.kStatorCurrentLimit = 40;

			kHoodServoConstants.kMaxForwardOutput = 12.0;
			kHoodServoConstants.kMaxReverseOutput = -12.0;

			kHoodServoConstants.kRampRate = 0.0;

			kHoodServoConstants.kNeutralMode = NeutralModeValue.Coast;

			kHoodEncoderConstants.encoder_type = FeedbackSensorSourceValue.FusedCANcoder;
			kHoodEncoderConstants.remote_encoder_port = Ports.HOOD_CANCODER;
			kHoodEncoderConstants.rotor_rotations_per_output = 314.0;
			kHoodEncoderConstants.remote_encoder_offset = isEpsilon ? 0.187 : -0.2861;
		}
	}

	public static final class ClimberConstants {

		public static final ServoMotorSubsystemConstants kClimberServoConstants = new ServoMotorSubsystemConstants();

		static {
			kClimberServoConstants.kName = "Climber";

			kClimberServoConstants.kMainConstants.id = Ports.CLIMBER_MAIN;
			kClimberServoConstants.kMainConstants.counterClockwisePositive = false;

			kClimberServoConstants.kFollowerConstants = new TalonFXConstants[1];
			kClimberServoConstants.kFollowerConstants[0] = new TalonFXConstants();
			kClimberServoConstants.kFollowerConstants[0].id = Ports.CLIMBER_FOLLOWER;

			kClimberServoConstants.kHomePosition = 0;
			kClimberServoConstants.kRotationsPerUnitDistance = 8.0 / (Units.inchesToMeters(0.655) * Math.PI);

			kClimberServoConstants.kKp = 1.0;
			kClimberServoConstants.kKi = 0.0;
			kClimberServoConstants.kKd = 0.0;

			kClimberServoConstants.kCruiseVelocity = 0.5; // inches of string length
			kClimberServoConstants.kAcceleration = 4.0;

			kClimberServoConstants.kMaxUnitsLimit = 0.429 + Conversions.inchesToMeters(3.0);
			kClimberServoConstants.kMinUnitsLimit = 0.0;

			kClimberServoConstants.kEnableSupplyCurrentLimit = true;
			kClimberServoConstants.kSupplyCurrentLimit = 60;
			kClimberServoConstants.kSupplyCurrentThreshold = 80;
			kClimberServoConstants.kSupplyCurrentTimeout = 0.5;

			kClimberServoConstants.kMaxForwardOutput = 12.0;
			kClimberServoConstants.kMaxReverseOutput = -12.0;

			kClimberServoConstants.kNeutralMode = NeutralModeValue.Brake;
		}
	}

	/**
	 * For standard deviations:
	 * Lower = trust more, higher = trust less
	 */
	public static final class PoseEstimatorConstants {
		public record CameraConfig(Pose3d offset, String config) {}
		;

		public static final Matrix<N2, N1> kStateStdDevs = VecBuilder.fill(Math.pow(0.2, 1), Math.pow(0.2, 1));
		public static final Matrix<N2, N1> kLocalMeasurementStdDevs =
				VecBuilder.fill(Math.pow(0.01, 1), Math.pow(0.01, 1));

		public static final Map<String, CameraConfig> cameras = Map.ofEntries(
				Map.entry(
						"right",
						new CameraConfig(
								new Pose3d(
										Units.inchesToMeters(3.071),
										Units.inchesToMeters(-6.81),
										Units.inchesToMeters(24.604),
										new Rotation3d(
												Units.degreesToRadians(0),
												Units.degreesToRadians(-18),
												Units.degreesToRadians(27))),
								"{\"camera_matrix\":[[910.3756558875847,0,809.2765926238984],[0,909.8129438903156,644.2713243574459],[0,0,1]],\"distortion_coefficients\":[0.06236712235474046,-0.062294270427656145,0.004664145480488657,-0.0006911909097633055,-0.00762026244976393],\"tag_size\":0.163,\"camera_settings\":{\"width\":1600,\"height\":1200,\"fps\":50}}")),
				Map.entry(
						"left",
						new CameraConfig(
								new Pose3d(
										Units.inchesToMeters(3.071),
										Units.inchesToMeters(6.81),
										Units.inchesToMeters(24.604),
										new Rotation3d(
												Units.degreesToRadians(0),
												Units.degreesToRadians(-18),
												Units.degreesToRadians(-27))),
								"{\"camera_matrix\":[[910.3756558875847,0,809.2765926238984],[0,909.8129438903156,644.2713243574459],[0,0,1]],\"distortion_coefficients\":[0.06236712235474046,-0.062294270427656145,0.004664145480488657,-0.0006911909097633055,-0.00762026244976393],\"tag_size\":0.163,\"camera_settings\":{\"width\":1600,\"height\":1200,\"fps\":50}}")));
	}

	public static VisionDeviceConstants kLeftVisionDevice = new VisionDeviceConstants(); // dot 13
	public static VisionDeviceConstants kRightVisionDevice = new VisionDeviceConstants(); // dot 12

	static {
		kLeftVisionDevice.kTableName = "PolarisLeft";
		kLeftVisionDevice.kRobotToCamera = new com.team254.lib.geometry.Transform2d(
				new Translation2d(Units.inchesToMeters(3.071), Units.inchesToMeters(7.325)),
				Rotation2d.fromDegrees(-27));

		kRightVisionDevice.kTableName = "PolarisRight";
		kRightVisionDevice.kRobotToCamera = new com.team254.lib.geometry.Transform2d(
				new Translation2d(Units.inchesToMeters(3.071), Units.inchesToMeters(-7.325)),
				Rotation2d.fromDegrees(27.0));
	}

	public static final class LinearServoConstants {
		public static final int kMaxVelocity = 32; // mm/s
		public static final int kMaxLength = 50; // mm
	}

	public static final class LimelightConstants {

		public static final double kNoteHeight = 0.0508;
		public static final double kNoteTargetOffset = 0.2;
		public static final double kMaxNoteTrackingDistance = 6.75;
		public static final double kNoteTrackEpsilon = 1.0;

		public static final String kName = "limelight";
		public static final Translation2d kRobotToCameraTranslation = new Translation2d(0.0, 0.0);
		public static final double kCameraHeightMeters = isEpsilon ? 0.59 : 0.65;
		public static final Rotation2d kCameraPitch = Rotation2d.fromDegrees(-18.0);
		public static final Rotation2d kCameraYaw = Rotation2d.fromDegrees(0.0);

		public static final GoalTracker.Configuration kNoteTrackerConstants = new GoalTracker.Configuration();

		static {
			kNoteTrackerConstants.kMaxTrackerDistance = 0.46;
			kNoteTrackerConstants.kMaxGoalTrackAge = 0.5;
			kNoteTrackerConstants.kCameraFrameRate = 30.0;
			kNoteTrackerConstants.kStabilityWeight = 1.0;
			kNoteTrackerConstants.kAgeWeight = 0.2;
			kNoteTrackerConstants.kSwitchingWeight = 0.2;
		}
	}
}
