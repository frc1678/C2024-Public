package com.team1678.frc2024.subsystems;

import com.team1678.frc2024.Constants;
import com.team1678.frc2024.Constants.ElevatorConstants;
import com.team1678.frc2024.loops.ILooper;
import com.team1678.frc2024.loops.Loop;
import com.team1678.frc2024.subsystems.servo.ServoMotorSubsystem;
import com.team1678.lib.requests.Request;
import com.team1678.lib.util.DelayedBoolean;
import com.team254.lib.util.Util;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator extends ServoMotorSubsystem {
	// Singleton instance
	public static Elevator mInstance;

	public static Elevator getInstance() {
		if (mInstance == null) {
			mInstance = new Elevator(ElevatorConstants.kElevatorServoConstants);
		}
		return mInstance;
	}

	private Elevator(final ServoMotorSubsystemConstants constants) {
		super(constants);
		zeroSensors();
		enableSoftLimits(false);
	}

	public static final double kExtensionHeight = 0.42;
	public static final double kTrapAmpHeight = 0.303;
	public static final double kRetractionHeight = 0.0;

	private boolean mHoming = false;
	private boolean mNeedsToHome = false;
	private final DelayedBoolean mHomingDelay =
			new DelayedBoolean(Timer.getFPGATimestamp(), Constants.ElevatorConstants.kHomingTimeout);

	@Override
	public void registerEnabledLoops(ILooper mEnabledLooper) {
		mEnabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {
				setOpenLoop(0.0);
			}

			@Override
			public void onLoop(double timestamp) {
				// Home if we're ready to home
				if (getSetpoint() == mConstants.kHomePosition && atHomingLocation() && mNeedsToHome && !mHoming) {
					setWantHome(true);
					// If we're done homing, we no longer need to home
				} else if (mControlState != ControlState.OPEN_LOOP && mHoming) {
					setWantHome(false);
				}
			}

			@Override
			public void onStop(double timestamp) {}
		});
	}

	public void setWantHome(boolean home) {
		mHoming = home;
		if (home) {
			mNeedsToHome = false;
			mHomingDelay.update(Timer.getFPGATimestamp(), false);
		}
	}

	@Override
	public void writePeriodicOutputs() {
		if (mHoming) {
			setOpenLoop(Constants.ElevatorConstants.kHomingOutput / mConstants.kMaxForwardOutput);
			if (mHomingDelay.update(
					Timer.getFPGATimestamp(),
					Math.abs(getVelocity()) < Constants.ElevatorConstants.kHomingVelocityWindow)) {
				zeroSensors();
				setSetpointMotionMagic(mConstants.kHomePosition);
				setWantHome(false);
				mNeedsToHome = false;
			}
		}

		super.writePeriodicOutputs();
	}

	@Override
	public void stop() {
		super.stop();
	}

	@Override
	public boolean checkSystem() {
		return false;
	}

	@Override
	public boolean atHomingLocation() {
		return mPeriodicIO.position_units - mConstants.kHomePosition < Constants.ElevatorConstants.kHomingZone;
	}

	@Override
	public void outputTelemetry() {
		SmartDashboard.putBoolean(mConstants.kName + "/Homing", mHoming);
		SmartDashboard.putBoolean(mConstants.kName + "/Within Homing Window", atHomingLocation());

		super.outputTelemetry();
	}

	/**
	 * @return New reqeust commanding the elevator to extend for Trap scoring.
	 */
	public Request fullExtendRequest() {
		return new Request() {

			@Override
			public void act() {
				setSetpointMotionMagic(kExtensionHeight);
				mNeedsToHome = true;
			}

			@Override
			public boolean isFinished() {
				return Util.epsilonEquals(getPosition(), kExtensionHeight, 0.1);
			}
		};
	}

	
	/**
	 * @return New reqeust commanding the elevator to extend for Amp scoring.
	 */
	public Request scoreHeightRequest() {
		return new Request() {

			@Override
			public void act() {
				setSetpointMotionMagic(kTrapAmpHeight);
				mNeedsToHome = true;
			}

			@Override
			public boolean isFinished() {
				return Util.epsilonEquals(getPosition(), kExtensionHeight, 0.1);
			}
		};
	}

	
	/**
	 * @return New reqeust commanding the elevator to retract.
	 */
	public Request retractRequest() {
		return new Request() {

			@Override
			public void act() {
				setSetpointMotionMagic(kRetractionHeight);
			}

			@Override
			public boolean isFinished() {
				return Util.epsilonEquals(getPosition(), kRetractionHeight, 0.1);
			}
		};
	}
}
