// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1678.frc2024;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team1678.frc2024.auto.AutoModeBase;
import com.team1678.frc2024.auto.AutoModeExecutor;
import com.team1678.frc2024.auto.AutoModeSelector;
import com.team1678.frc2024.controlboard.ControlBoard;
import com.team1678.frc2024.controlboard.DriverControls;
import com.team1678.frc2024.loops.CrashTracker;
import com.team1678.frc2024.loops.Looper;
import com.team1678.frc2024.paths.TrajectoryGenerator;
import com.team1678.frc2024.subsystems.AmpRollers;
import com.team1678.frc2024.subsystems.Cancoders;
import com.team1678.frc2024.subsystems.Climber;
import com.team1678.frc2024.subsystems.Drive;
import com.team1678.frc2024.subsystems.Elevator;
import com.team1678.frc2024.subsystems.Feeder;
import com.team1678.frc2024.subsystems.Hood;
import com.team1678.frc2024.subsystems.IntakeDeploy;
import com.team1678.frc2024.subsystems.IntakeRollers;
import com.team1678.frc2024.subsystems.LEDs;
import com.team1678.frc2024.subsystems.Serializer;
import com.team1678.frc2024.subsystems.Shooter;
import com.team1678.frc2024.subsystems.Superstructure;
import com.team1678.frc2024.subsystems.limelight.Limelight;
import com.team1678.frc2024.subsystems.limelight.Limelight.Pipeline;
import com.team1678.frc2024.subsystems.vision.VisionDeviceManager;
import com.team1678.lib.Util;
import com.team1678.lib.logger.LogUtil;
import com.team1678.lib.sim.PhysicsSim;
import com.team1678.lib.swerve.ChassisSpeeds;
import com.team1678.lib.wpi.TimedRobot;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithMotion;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.timing.TimedState;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.List;
import java.util.Optional;

public class Robot extends TimedRobot {

	// util instances
	private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();
	private final ControlBoard mControlBoard = ControlBoard.getInstance();
	private final DriverControls mDriverControls = new DriverControls();

	// the boss
	private final Superstructure mSuperstructure = Superstructure.getInstance();

	// subsystem instances
	private Drive mDrive;
	private IntakeRollers mIntakeRollers;
	private IntakeDeploy mIntakeDeploy;
	private AmpRollers mAmpRollers;
	private Elevator mElevator;
	private Serializer mSerializer;
	private Feeder mFeeder;
	private Shooter mShooter;
	private Hood mHood;
	private Climber mClimber;
	private LEDs mLEDs;
	private Cancoders mCancoders;

	// vision
	private final VisionDeviceManager mVisionDevices = VisionDeviceManager.getInstance();

	// limelight
	private final Limelight mLimelight = Limelight.getInstance();

	// enabled and disabled loopers
	private final Looper mEnabledLooper = new Looper();
	private final Looper mDisabledLooper = new Looper();

	// auto instances
	private AutoModeExecutor mAutoModeExecutor;
	public static final AutoModeSelector mAutoModeSelector = new AutoModeSelector();
	public static final AutoModeSelector mThreeNoteNoteSelector = new AutoModeSelector();
	public static boolean is_red_alliance = false;
	public static String serial;

	double disable_enter_time = 0.0;

	static {
		if (Robot.isReal()) {
			serial = System.getenv("serialnum");
		} else {
			serial = "";
		}
		Constants.isComp = serial.startsWith(Constants.kCompSerial);
		Constants.isEpsilon = serial.startsWith(Constants.kEpsilonSerial);
	}

	public Robot() {
		CrashTracker.logRobotConstruction();
	}

	@Override
	public void robotInit() {
		try {
			mDrive = Drive.getInstance();
			mIntakeRollers = IntakeRollers.getInstance();
			mIntakeDeploy = IntakeDeploy.getInstance();
			mAmpRollers = AmpRollers.getInstance();
			mElevator = Elevator.getInstance();
			mSerializer = Serializer.getInstance();
			mFeeder = Feeder.getInstance();
			mShooter = Shooter.getInstance();
			mHood = Hood.getInstance();
			mClimber = Climber.getInstance();
			mLEDs = LEDs.getInstance();
			mCancoders = Cancoders.getInstance();
			CrashTracker.logRobotInit();

			LiveWindow.disableAllTelemetry();

			SmartDashboard.putBoolean("Is Comp", Constants.isComp);
			SmartDashboard.putBoolean("Is Epsilon", Constants.isEpsilon);

			if (!Constants.isComp && !Constants.isEpsilon) {
				SmartDashboard.putString("Comp Serial", Constants.kCompSerial);
				SmartDashboard.putString("Epsilon Serial", Constants.kEpsilonSerial);
				SmartDashboard.putString("Serial Number", serial);
			}

			if (Robot.isReal()) {
				mCancoders = Cancoders.getInstance();
				double startInitTs = Timer.getFPGATimestamp();
				System.out.println("* Starting to init Cancoders at ts " + startInitTs);
				while (Timer.getFPGATimestamp() - startInitTs < Constants.SwerveConstants.kCancoderBootAllowanceSeconds
						&& !mCancoders.allHaveBeenInitialized()) {
					Timer.delay(0.1);
				}
				System.out.println(
						"* Cancoders all initialized: Took " + (Timer.getFPGATimestamp() - startInitTs) + " seconds");
			}

			mDrive.resetModulesToAbsolute();

			// spotless:off
			mSubsystemManager.setSubsystems(
				mDrive, 
				mSuperstructure,
				mIntakeDeploy,
				mIntakeRollers,
				mAmpRollers,
				mElevator,
				mSerializer,
				mFeeder,
				mShooter,
				mHood,
				mClimber,
				mLEDs,
				mVisionDevices,
				mLimelight
			);
			// spotless:on

			mSubsystemManager.registerEnabledLoops(mEnabledLooper);
			mSubsystemManager.registerDisabledLoops(mDisabledLooper);

			TrajectoryGenerator.getInstance().generateTrajectories();
			RobotState.getInstance().resetKalman();
			mDrive.setNeutralBrake(true);

			RobotController.setBrownoutVoltage(5.5);

			DataLogManager.start();

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void robotPeriodic() {
		mEnabledLooper.outputToSmartDashboard();
		mSubsystemManager.outputLoopTimes();
	}

	@Override
	public void autonomousInit() {
		if (mVisionDevices.getMovingAverageRead() != null) {
			mDrive.zeroGyro(mVisionDevices.getMovingAverageRead());
		}
		Superstructure.getInstance().setFerry(false);
		RobotState.getInstance().setIsInAuto(true);
		mDisabledLooper.stop();
		mEnabledLooper.start();
		mAutoModeExecutor.start();
	}

	@Override
	public void autonomousPeriodic() {}

	@Override
	public void autonomousExit() {}

	@Override
	public void teleopInit() {
		try {
			RobotState.getInstance().setIsInAuto(false);
			mDrive.feedTeleopSetpoint(new ChassisSpeeds(0.0, 0.0, 0.0));
			VisionDeviceManager.setDisableVision(false);
			mDisabledLooper.stop();
			mEnabledLooper.start();

			mSuperstructure.tuckState();
			mSuperstructure.idleState();

			mLimelight.setPipeline(Pipeline.TELEOP);
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void teleopPeriodic() {
		try {
			mControlBoard.update();

			/* Drive */
			if (mControlBoard.zeroGyro()) {
				System.out.println("Zeroing gyro!");
				mDrive.zeroGyro(FieldLayout.handleAllianceFlip(new Rotation2d(), is_red_alliance)
						.getDegrees());
				mDrive.resetModulesToAbsolute();
				mDrive.resetOdometry(new Pose2d());
			}

			mDrive.feedTeleopSetpoint(ChassisSpeeds.fromFieldRelativeSpeeds(
					mControlBoard.getSwerveTranslation().x(),
					mControlBoard.getSwerveTranslation().y(),
					mControlBoard.getSwerveRotation(),
					Util.robotToFieldRelative(mDrive.getHeading(), is_red_alliance)));

			mDriverControls.oneControllerMode();

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void disabledInit() {
		try {
			VisionDeviceManager.setDisableVision(false);
			CrashTracker.logDisabledInit();
			mEnabledLooper.stop();
			mDisabledLooper.start();
			disable_enter_time = Timer.getFPGATimestamp();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}

		if (mAutoModeExecutor != null) {
			mAutoModeExecutor.stop();
		}

		// Reset all auto mode state.
		mAutoModeSelector.reset();
		mAutoModeSelector.updateModeCreator(false);
		mAutoModeExecutor = new AutoModeExecutor();
		mLimelight.setPipeline(is_red_alliance ? Pipeline.AUTO_RED : Pipeline.AUTO_BLUE);
	}

	@Override
	public void disabledPeriodic() {
		try {
			boolean alliance_changed = false;
			if (DriverStation.getAlliance().isPresent()) {
				if (DriverStation.getAlliance().get() == Alliance.Red) {
					alliance_changed = !is_red_alliance;
					is_red_alliance = true;
				} else if (DriverStation.getAlliance().get() == Alliance.Blue) {
					alliance_changed = is_red_alliance;
					is_red_alliance = false;
				}
			} else {
				alliance_changed = true;
			}

			if (Timer.getFPGATimestamp() - disable_enter_time > 5.0) {
				mHood.setNeutralMode(NeutralModeValue.Coast);
				disable_enter_time = Double.POSITIVE_INFINITY;
			}

			if (alliance_changed) {
				System.out.println("Alliance changed! Requesting trajectory regeneration!");
				TrajectoryGenerator.getInstance().forceRegenerateTrajectories(is_red_alliance);
				mLimelight.setPipeline(is_red_alliance ? Pipeline.AUTO_RED : Pipeline.AUTO_BLUE);
			}

			mAutoModeSelector.updateModeCreator(alliance_changed);
			Optional<AutoModeBase> autoMode = mAutoModeSelector.getAutoMode();

			if (autoMode.isPresent() && (autoMode.get() != mAutoModeExecutor.getAutoMode())) {
				mAutoModeExecutor.setAutoMode(autoMode.get());
			}

			List<Trajectory<TimedState<Pose2dWithMotion>>> paths =
					autoMode.get().getPaths();
			for (int i = 0; i < paths.size(); i++) {
				LogUtil.recordTrajectory(String.format("Paths/Path %d", i), paths.get(i));
			}

			if (mControlBoard.driver.getBButton()) {
				RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());
			}

			SmartDashboard.putNumber("Vision Heading/Average", mVisionDevices.getMovingAverageRead());

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void testInit() {
		try {
			mDisabledLooper.stop();
			mEnabledLooper.stop();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void testPeriodic() {}

	@Override
	public void simulationPeriodic() {
		PhysicsSim.getInstance().run();
	}
}
