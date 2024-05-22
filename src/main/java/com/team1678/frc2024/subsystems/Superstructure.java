package com.team1678.frc2024.subsystems;

import com.team1678.frc2024.Constants;
import com.team1678.frc2024.FieldLayout;
import com.team1678.frc2024.Ports;
import com.team1678.frc2024.Robot;
import com.team1678.frc2024.RobotState;
import com.team1678.frc2024.led.TimedLEDState;
import com.team1678.frc2024.loops.ILooper;
import com.team1678.frc2024.loops.Loop;
import com.team1678.frc2024.planners.ElevatorMotionPlanner;
import com.team1678.frc2024.shooting.FerryUtil;
import com.team1678.frc2024.shooting.ShootingUtil;
import com.team1678.frc2024.subsystems.Serializer.State;
import com.team1678.lib.TunableNumber;
import com.team1678.lib.drivers.BeamBreak;
import com.team1678.lib.requests.IfRequest;
import com.team1678.lib.requests.LambdaRequest;
import com.team1678.lib.requests.ParallelRequest;
import com.team1678.lib.requests.Request;
import com.team1678.lib.requests.SequentialRequest;
import com.team1678.lib.requests.WaitRequest;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.util.TimeDelayedBoolean;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.List;

public class Superstructure extends Subsystem {

	private static Superstructure mInstance;

	public static synchronized Superstructure getInstance() {
		if (mInstance == null) {
			mInstance = new Superstructure();
		}

		return mInstance;
	}

	// Request tracking variables
	private Request activeRequest = null;
	private ArrayList<Request> queuedRequests = new ArrayList<>(0);
	private boolean hasNewRequest = false;
	private boolean allRequestsComplete = false;

	// Subsystems
	private final Climber mClimber = Climber.getInstance();
	private final IntakeRollers mIntakeRollers = IntakeRollers.getInstance();
	private final IntakeDeploy mIntakeDeploy = IntakeDeploy.getInstance();
	private final Serializer mSerializer = Serializer.getInstance();
	private final Feeder mFeeder = Feeder.getInstance();
	private final Elevator mElevator = Elevator.getInstance();
	private final AmpRollers mAmpRollers = AmpRollers.getInstance();
	private final Shooter mShooter = Shooter.getInstance();
	private final Hood mHood = Hood.getInstance();

	// LEDs
	private final LEDs mLEDs = LEDs.getInstance();
	private TimedLEDState mHeldState = TimedLEDState.NOTE_HELD_SHOT;

	// Beam breaks
	private BeamBreak mSerializerBreak = new BeamBreak(Ports.SERIALIZER_BREAK);
	private BeamBreak mFeederBreak = new BeamBreak(Ports.FEEDER_BREAK);
	private BeamBreak mAmpBreak = new BeamBreak(Ports.AMP_BREAK);

	private ElevatorMotionPlanner mElevatorMotionPlanner = new ElevatorMotionPlanner();

	// Target tracking
	private Drive mDrive = Drive.getInstance();
	private double mDistanceToTarget = 0.0;
	private double mAngularErrToTarget = 0.0;

	// Manual param tuning
	public final boolean kUseSmartdash = false;
	public TunableNumber kCurveTuner = new TunableNumber("FiringParams/ManualCurveTune", 0.0, true);
	public TunableNumber kSkewTuner = new TunableNumber("FiringParams/ManualSkewTune", 0.0, true);
	public TunableNumber mHoodTuner = new TunableNumber("FiringParams/ManualHoodTune", 0.0, true);
	public TunableNumber mRPMTuner = new TunableNumber("FiringParams/ManualRPMTune", 0.0, true);

	// Trackers
	private boolean CLIMB_MODE = false;
	private boolean PREP = false;
	private boolean FERRY_SHOT = false;
	private boolean WANTS_SPINDOWN = false;

	public boolean requestsCompleted() {
		return allRequestsComplete;
	}

	public void request(Request r) {
		setActiveRequest(r);
		clearRequestQueue();
	}

	private void setActiveRequest(Request request) {
		activeRequest = request;
		hasNewRequest = true;
		allRequestsComplete = false;
	}

	private void clearRequestQueue() {
		queuedRequests.clear();
	}

	private void setRequestQueue(List<Request> requests) {
		clearRequestQueue();
		for (Request req : requests) {
			queuedRequests.add(req);
		}
	}

	private void setRequestQueue(Request activeRequest, ArrayList<Request> requests) {
		request(activeRequest);
		setRequestQueue(requests);
	}

	private void addRequestToQueue(Request req) {
		queuedRequests.add(req);
	}

	@Override
	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {
				clearRequestQueue();
			}

			@Override
			public void onLoop(double timestamp) {
				try {
					if (hasNewRequest && activeRequest != null) {
						activeRequest.act();
						hasNewRequest = false;
					}

					if (activeRequest == null) {
						if (queuedRequests.isEmpty()) {
							allRequestsComplete = true;
						} else {
							request(queuedRequests.remove(0));
						}
					} else if (activeRequest.isFinished()) {
						activeRequest = null;
					}

					updateShootingSetpoints();
				} catch (Exception e) {
					e.printStackTrace();
				}
			}

			@Override
			public void onStop(double timestamp) {
				stop();
			}
		});
	}

	@Override
	public void stop() {
		activeRequest = null;
		clearRequestQueue();
	}

	@Override
	public boolean checkSystem() {
		return false;
	}

	@Override
	public void readPeriodicInputs() {
		mAmpBreak.update();
		mFeederBreak.update();
		mSerializerBreak.update();
	}

	@Override
	public void outputTelemetry() {
		SmartDashboard.putBoolean("BeamBreaks/Amp Break", mAmpBreak.get());
		SmartDashboard.putBoolean("BeamBreaks/Serializer Break", mSerializerBreak.get());
		SmartDashboard.putBoolean("BeamBreaks/Feeder Break", mFeederBreak.get());

		SmartDashboard.putBoolean("Ferry Shot", FERRY_SHOT);

		SmartDashboard.putBoolean(
				"Superstructure/Elevator up met",
				mElevatorMotionPlanner.elevatorClearUp().met());
		SmartDashboard.putBoolean(
				"Superstructure/Elevator down met",
				mElevatorMotionPlanner.elevatorClearDown().met());
		SmartDashboard.putBoolean(
				"Superstructure/Intake clear",
				mElevatorMotionPlanner.intakeClear().met());

		SmartDashboard.putNumber("FiringParams/Angular Err To Target", mAngularErrToTarget);
	}

	/* Superstructure functions */

	// spotless:off 
	public double getAngularErrToTarget() {
		return mAngularErrToTarget;
	}

	public boolean getFerryShotMode() {
		return FERRY_SHOT;
	}

	/**
	 * Utilize Robot Pose, distance to target, and 
	 * polls regression values to determine 
	 * setpoints for Hood and Shooter.
	 */
	private void updateShootingSetpoints() {
		if (CLIMB_MODE) {
			mShooter.setVelocity(0.0, 0.0);
			mHood.setSetpointMotionMagic(54.0);
			return;
		}

		Pose2d robot_pose = mDrive.getPose();

		double shooter_setpoint;
		double hood_setpoint;
		Rotation2d target_drive_heading;

		if (FERRY_SHOT) {
			double[] shooting_params = FerryUtil.getFerryShotParameters(robot_pose, Robot.is_red_alliance);
			mDistanceToTarget = shooting_params[0];
			hood_setpoint = shooting_params[1];
			shooter_setpoint = shooting_params[2];
			target_drive_heading = Rotation2d.fromDegrees(shooting_params[3]);
		} else {
			double[] shooting_params = ShootingUtil.getSpeakerShotParameters(robot_pose, RobotState.getInstance().getMeasuredVelocity(), Robot.is_red_alliance);
			mDistanceToTarget = shooting_params[0];
			hood_setpoint = shooting_params[1];
			shooter_setpoint = shooting_params[2];
			target_drive_heading = Rotation2d.fromDegrees(shooting_params[3]);

			if (!PREP || mAmpBreak.get() || mDistanceToTarget > 10.4) {
				shooter_setpoint = 120.0;
				hood_setpoint = 50.0;
			}

			if (WANTS_SPINDOWN && mDistanceToTarget > FieldLayout.kWingX) {
				shooter_setpoint = 500.0;
			}
		}

		if (mHood.isHoming()) {
			mShooter.setVelocity(0.0, 0.0);
		} else {
			mShooter.setVelocity(shooter_setpoint, shooter_setpoint);
			mHood.setSetpointMotionMagic(hood_setpoint);
		}

		SmartDashboard.putNumber("FiringParams/Distance to target", mDistanceToTarget);
		SmartDashboard.putNumber("FiringParams/HoodError", hood_setpoint - mHood.getPosition());
		SmartDashboard.putNumber("FiringParams/TargetHeading", target_drive_heading.getDegrees());
		SmartDashboard.putNumber("FiringParams/CurrentHeading", mDrive.getHeading().getDegrees());

		mDrive.feedTrackingSetpoint(target_drive_heading);
		mAngularErrToTarget = target_drive_heading.rotateBy(mDrive.getHeading().inverse()).getDegrees();
	}
	
	/**
	 * BeamBreak Sensor reading.
	 * 
	 * @param mBreak BeamBreak Sensor.
	 * @param target_state If wanted reading is true (broken) or false (not broken).
	 * 
	 * @return Boolean for if target state is acheived. 
	 */
	private Request breakWait(BeamBreak mBreak, boolean target_state) {
		return new Request() {

			@Override
			public void act() {}

			@Override
			public boolean isFinished() {
				return mBreak.get() == target_state;
			}
		};
	}

	/**
	 * Debounced BeamBreak Sensor reading.
	 * 
	 * @param mBreak BeamBreak Sensor.
	 * @param target_state If wanted reading is true (broken) or false (not broken).
	 * @param delayed_wait_seconds Debounces time from a BeamBreak Sensor. 
	 * 
	 * @return Boolean for if target state is acheived after debouncing the signal. 
	 */
	private Request breakWait(BeamBreak mBreak, boolean target_state, double delayed_wait_seconds) {
		return new Request() {

			TimeDelayedBoolean timeout = new TimeDelayedBoolean();

			@Override
			public void act() {}

			@Override
			public boolean isFinished() {
				return timeout.update(mBreak.get() == target_state, delayed_wait_seconds);
			}
		};
	}

	/**
	 * Update state of LEDs based on BeamBreak readings.
	 */
	private Request updateLEDsRequest() {
		return new Request() {

			@Override
			public void act() {
				updateLEDs();
			}

			@Override
			public boolean isFinished() {
				return true;
			}
		};
	}

	/**
	 * Update state of LEDs based on BeamBreak readings.
	 */
	private void updateLEDs() {
		if (mAmpBreak.get()) {
			mLEDs.applyStates(TimedLEDState.ELEVATOR_LOADED);
		} else if (mFeederBreak.get()) {
			mLEDs.applyStates(mHeldState);
		} else {
			mLEDs.applyStates(TimedLEDState.OFF);
		}
	}

	public boolean getFeederBreak() {
		return mFeederBreak.get();
	}

	public boolean getAmpBreak() {
		return mAmpBreak.get();
	}

	public boolean getSerializerBreak() {
		return mSerializerBreak.get();
	}

	/**
	 * Stop Intake Rollers, Serializer, Feeder, and Amp Rollers.
	 */
	private Request idleRequest() {
		return new ParallelRequest(
			mIntakeRollers.stateRequest(IntakeRollers.State.IDLE),
			mSerializer.stateRequest(Serializer.State.IDLE),
			mFeeder.stateRequest(Feeder.State.IDLE),
			mAmpRollers.stateRequest(AmpRollers.State.IDLE)
		);
	}

	/* ControlBoard Facing States */
	public void setWantClimbMode(boolean enable) {
		if (CLIMB_MODE != enable) {
			CLIMB_MODE = enable;			
		}
	}

	public void setWantPrep(boolean enable) {
		WANTS_SPINDOWN = false;
		if (PREP != enable) {
			PREP = enable;
		}
	}

	public void toggleFerry() {
		setFerry(!FERRY_SHOT);
	}
	
	public void setFerry(boolean enable) {
		FERRY_SHOT = enable;
		if (FERRY_SHOT) {
			mHeldState = TimedLEDState.NOTE_HELD_FERRY;
		} else {
			mHeldState = TimedLEDState.NOTE_HELD_SHOT;
		}
		updateLEDs();
	}

	/**
	 * Stop Intake Rollers, Serializer, Feeder, and Amp Rollers.
	 */
	public void idleState() {
		request(idleRequest());
	}

	/**
	 * Tuck Elevator and Intake Deploy and stop Intake Rollers,
	 * Serializer, and Amp Rollers.
	 */
	public void tuckState() {
		request(new ParallelRequest(
			mIntakeRollers.stateRequest(IntakeRollers.State.IDLE),
			mAmpRollers.stateRequest(AmpRollers.State.IDLE),
			mSerializer.stateRequest(State.IDLE),
			new SequentialRequest(
				mElevatorMotionPlanner.getRetractPlan(),
				mIntakeDeploy.tuckRequest()
			),
			updateLEDsRequest()
		));
	}

	/**
	 * Shoot note and updates LEDs.
	 */
	public void fireState() {
		request(new SequentialRequest(
			new LambdaRequest(() -> WANTS_SPINDOWN = false),
			idleRequest(),
			mShooter.waitRequest(),
			mLEDs.stateRequest(TimedLEDState.FIRING),
			mSerializer.stateRequest(Serializer.State.INTAKE),
			mFeeder.stateRequest(Feeder.State.SHOOTER_FEED),
			breakWait(mFeederBreak, false),
			new WaitRequest(0.2),
			mLEDs.stateRequest(TimedLEDState.OFF),
			idleRequest(),
			new LambdaRequest(() -> WANTS_SPINDOWN = true)
		));
	}

	/**
	 * Extend Elevator to amp scoring height.
	 */
	public void elevatorAmpExtendTransition() {
		request(new SequentialRequest(
			breakWait(mAmpBreak, true),
			idleRequest(),
			mElevatorMotionPlanner.getAmpExtendPlan(),
			mIntakeDeploy.tuckRequest()
		));
	}

	/**
	 * Extend Elevator to its max height.
	 */
	public void elevatorFullExtendTransition() {
		request(new SequentialRequest(
			idleRequest(),
			mElevatorMotionPlanner.getFullExtendPlan()
		));
	}

	/**
	 * If Amp BeamBreak is read, spin Amp Rollers forward 
	 * then retract Elevator and stow Intake Deploy. 
	 */
	public void ampScoreTransition() {
		request(new SequentialRequest(
			breakWait(mAmpBreak, true),
			idleRequest(),
			mAmpRollers.stateRequest(AmpRollers.State.FORWARD),
			breakWait(mAmpBreak, false),
			new WaitRequest(0.2),
			idleRequest(),
			mIntakeDeploy.clearRequest(),
			mElevator.retractRequest(),
			mIntakeDeploy.tuckRequest(),
			mLEDs.stateRequest
			(TimedLEDState.OFF)
		));
	}
	/**
	 * Specifically for trap position. If Amp BeamBreak 
	 * is read, spin Amp Rollers forward.
	 */
	public void trapScoreTransition() {
		request(new SequentialRequest(
			breakWait(mAmpBreak, true),
			idleRequest(),
			mAmpRollers.stateRequest(AmpRollers.State.FORWARD),
			breakWait(mAmpBreak, false),
			new WaitRequest(3.0),
			idleRequest(),
			mLEDs.stateRequest(TimedLEDState.OFF)));
	}

	/**
	 * Eject notes by reversing Intake Rollers, Serializer,
	 * Feeder, and Amp Rollers.
	 */
	public void exhaustState() {
		request(new ParallelRequest(
			mIntakeDeploy.unjamRequest(),
			mIntakeRollers.stateRequest(IntakeRollers.State.EXHAUST),
			mSerializer.stateRequest(Serializer.State.EXHAUST),
			mFeeder.stateRequest(Feeder.State.AMP_FEED),
			mAmpRollers.stateRequest(AmpRollers.State.FORWARD)));
	}

	/**
	 * Puts Intake Deploy in intaking position, then
	 * wait for a Feeder BeamBreak reading before
	 * stowing Intake Deploy.
	 */
	public void intakeToHoldTransition() {
		request(new SequentialRequest(
			breakWait(mSerializerBreak, false),
			new LambdaRequest(() -> WANTS_SPINDOWN = false),
			updateLEDsRequest(),
			mIntakeDeploy.deployRequest(),
			mIntakeRollers.stateRequest(IntakeRollers.State.INTAKING),
			mSerializer.stateRequest(Serializer.State.INTAKE),
			mFeeder.stateRequest(Feeder.State.IDLE),
			new ParallelRequest(
				new SequentialRequest(
					breakWait(mFeederBreak, true, 0.0),
					mIntakeRollers.stateRequest(IntakeRollers.State.EXHAUST),
					mIntakeDeploy.clearRequest(),
					new WaitRequest(0.5),
					mIntakeRollers.stateRequest(IntakeRollers.State.IDLE),
					mIntakeDeploy.tuckRequest()
				),
				new SequentialRequest(
					breakWait(mFeederBreak, true),
					mSerializer.stateRequest(Serializer.State.IDLE),
					mFeeder.stateRequest(Feeder.State.IDLE),
					mLEDs.stateRequest(mHeldState)
				)
			)
		));
	}

	/**
	 * Take a note from Feeder and transfer it to
	 * the amp/trap mechanism. 
	 */
	public void holdToAmpTransition() {
		request(new SequentialRequest(
			idleRequest(),
			mLEDs.stateRequest(TimedLEDState.ELEVATOR_LOADING),
			mIntakeDeploy.clearRequest(),
			mElevator.retractRequest(),

			new IfRequest(Constants.isCompSupplier(), new SequentialRequest(
				mSerializer.stateRequest(Serializer.State.SLOW_EXHAUST),
				breakWait(mFeederBreak, false)
			)),
			
			mSerializer.stateRequest(Serializer.State.IDLE),
			mAmpRollers.stateRequest(AmpRollers.State.FORWARD),
			mFeeder.stateRequest(Feeder.State.AMP_FEED),
			mSerializer.stateRequest(Serializer.State.SLOW_FEED),
			breakWait(mAmpBreak, true),
			idleRequest(),
			mLEDs.stateRequest(TimedLEDState.ELEVATOR_LOADED),
			mIntakeDeploy.tuckRequest()
		));
	}

	/**
	 * Put Intake Deploy in intaking position, then
	 * wait for a Feeder BeamBreak reading before feeding 
	 * acquired note into the amp/trap mechanism. 
	 */
	public void intakeToAmpTransition() {
		request(new SequentialRequest(
			breakWait(mSerializerBreak, false),
			idleRequest(),
			mLEDs.stateRequest(TimedLEDState.ELEVATOR_LOADING),
			mIntakeDeploy.deployRequest(),
			mElevator.retractRequest(),
			mIntakeRollers.stateRequest(IntakeRollers.State.INTAKING),
			mAmpRollers.stateRequest(AmpRollers.State.FORWARD),
			mFeeder.stateRequest(Feeder.State.AMP_FEED),
			mSerializer.stateRequest(Serializer.State.INTAKE),
			breakWait(mAmpBreak, true),
			mLEDs.stateRequest(TimedLEDState.ELEVATOR_LOADED),
			idleRequest(),
			mIntakeRollers.stateRequest(IntakeRollers.State.EXHAUST),
			mIntakeDeploy.clearRequest(),
			new WaitRequest(0.5),
			mIntakeRollers.stateRequest(IntakeRollers.State.IDLE),
			mIntakeDeploy.tuckRequest()
		));
	}

	/**
	 * Reverse Amp Rollers and put Intake Deploy in 
	 * intaking position to create a clear path to eject a note 
	 * in the amp/trap mechanism. 
	 */
	public void ampUnjam() {
		request(new SequentialRequest(
			mIntakeDeploy.deployRequest(),
			mAmpRollers.stateRequest(AmpRollers.State.REVERSE)
		));
	}

	/**
	 * Continuously run Intake Rollers, Serializer, Feeder, and Shooter. 
	 * Put the Intake Deploy in intaking position and never 
	 * stow it. 
	 */
	public void continuousShootState() {
		request(new SequentialRequest(
			mShooter.waitRequest(),
			mLEDs.stateRequest(TimedLEDState.FIRING),
			mFeeder.stateRequest(Feeder.State.SHOOTER_FEED),
			mIntakeRollers.stateRequest(IntakeRollers.State.INTAKING),
			mSerializer.stateRequest(Serializer.State.INTAKE),
			mIntakeDeploy.deployRequest()
		));
	}

	/**
	 * Continuously run Intake Rollers, Serializer, Feeder, and Shooter. 
	 * Put Intake Deploy in intaking position and never stow.
	 * Transfer notes into the Shooter slowly in order
	 * to increase consistency of note shots in auto. 
	 */
	public void slowContinuousShotState() {
		request(new SequentialRequest(
			mIntakeRollers.stateRequest(IntakeRollers.State.INTAKING),
			mShooter.waitRequest(),
			mLEDs.stateRequest(TimedLEDState.FIRING),
			mFeeder.stateRequest(Feeder.State.SLOW_FEED),
			mSerializer.stateRequest(Serializer.State.INTAKE),
			mIntakeDeploy.deployRequest()
		));
	}

	public void releaseElevator() {
		mElevator.setOpenLoop(0.0);
	}
	// spotless:on
}
