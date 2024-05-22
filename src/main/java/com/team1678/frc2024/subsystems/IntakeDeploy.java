package com.team1678.frc2024.subsystems;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team1678.frc2024.Constants;
import com.team1678.frc2024.Constants.IntakeDeployConstants;
import com.team1678.frc2024.loops.ILooper;
import com.team1678.frc2024.loops.Loop;
import com.team1678.frc2024.subsystems.servo.ServoMotorSubsystem;
import com.team1678.lib.Util;
import com.team1678.lib.requests.Request;
import com.team1678.lib.util.DelayedBoolean;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeDeploy extends ServoMotorSubsystem {
	public static IntakeDeploy mInstance;

	public static IntakeDeploy getInstance() {
		if (mInstance == null) {
			mInstance = new IntakeDeploy(IntakeDeployConstants.kDeployServoConstants);
		}
		return mInstance;
	}

	public static final double kDeployAngle = Constants.isEpsilon ? 15.0 : 6.5;
	public static final double kClearAngle = 96.0;
	public static final double kUnjamAngle = 30.0;
	public static final double kStowAngle = IntakeDeployConstants.kDeployServoConstants.kHomePosition;

	private boolean mHoming = false;
	private boolean mNeedsToHome = false;
	private final DelayedBoolean mHomingDelay =
			new DelayedBoolean(Timer.getFPGATimestamp(), Constants.IntakeDeployConstants.kHomingTimeout);

	public IntakeDeploy(final ServoMotorSubsystemConstants constants) {
		super(constants);
		mMain.setPosition(homeAwareUnitsToRotations(120.0));
		enableSoftLimits(false);
		setSetpointMotionMagic(kStowAngle);
	}

	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {}

			@Override
			public void onLoop(double timestamp) {
				if (getSetpoint() == mConstants.kHomePosition && atHomingLocation() && mNeedsToHome && !mHoming) {
					setWantHome(true);
				} else if (mControlState != ControlState.OPEN_LOOP && mHoming) {
					setWantHome(false);
				}
			}

			@Override
			public void onStop(double timestamp) {
				setNeutralMode(NeutralModeValue.Brake);
			}
		});
	}

	/**
	 * Runs intake deploy rehoming sequence.
	 * @param home Enable/Disable rehome sequence.
	 */
	public void setWantHome(boolean home) {
		mHoming = home;

		if (mHoming) {
			mNeedsToHome = false;
		}
	}

	@Override
	public synchronized void writePeriodicOutputs() {
		if (mHoming) {
			setOpenLoop(Constants.IntakeDeployConstants.kHomingOutput / mConstants.kMaxForwardOutput);
			if (mHomingDelay.update(
					Timer.getFPGATimestamp(),
					Math.abs(getVelocity()) < Constants.IntakeDeployConstants.kHomingVelocityWindow)) {
				zeroSensors();
				mHasBeenZeroed = true;
				setSetpointMotionMagic(mConstants.kHomePosition);
				mHoming = false;
			}
		}

		super.writePeriodicOutputs();
	}

	@Override
	public void outputTelemetry() {
		SmartDashboard.putBoolean(mConstants.kName + "/Homing", mHoming);
		SmartDashboard.putBoolean(mConstants.kName + "/Within Homing Window", atHomingLocation());
		super.outputTelemetry();
	}

	@Override
	public void stop() {}

	@Override
	public boolean checkSystem() {
		return false;
	}

	@Override
	public boolean atHomingLocation() {
		// Check if intake is  within 7.0 degrees of homing position
		return mPeriodicIO.position_units - mConstants.kHomePosition > -Constants.IntakeDeployConstants.kHomingZone;
	}

	/**
	 * @return Request to move intake to deploy angle to pick up notes off the ground.
	 */
	public Request deployRequest() {
		return new Request() {
			@Override
			public void act() {
				setSetpointMotionMagic(kDeployAngle);
				mNeedsToHome = true;
			}

			@Override
			public boolean isFinished() {
				return Util.epsilonEquals(getPosition(), kDeployAngle, 20.0);
			}
		};
	}

	/**
	 * @return Request to move intake to stow angle.
	 */
	public Request tuckRequest() {
		return new Request() {
			@Override
			public void act() {
				setSetpointMotionMagic(kStowAngle);
			}

			@Override
			public boolean isFinished() {
				return Util.epsilonEquals(getPosition(), kStowAngle, 4.0);
			}
		};
	}

	/**
	 * @return Request to move intake in order to avoid collisions with the elevator.
	 */
	public Request clearRequest() {
		return new Request() {
			@Override
			public void act() {
				setSetpointMotionMagic(kClearAngle);
			}

			@Override
			public boolean isFinished() {
				return Util.epsilonEquals(getPosition(), kClearAngle, 4.0);
			}
		};
	}

	/**
	 * @return Request to move intake in order to exhaust notes forwards, out of the robot.
	 */
	public Request unjamRequest() {
		return new Request() {
			@Override
			public void act() {
				setSetpointMotionMagic(kUnjamAngle);
			}

			@Override
			public boolean isFinished() {
				return Util.epsilonEquals(getPosition(), kUnjamAngle, 4.0);
			}
		};
	}
}
