package com.team1678.frc2024.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team1678.frc2024.Constants.IntakeRollerConstants;
import com.team1678.frc2024.Ports;
import com.team1678.frc2024.loops.ILooper;
import com.team1678.frc2024.loops.Loop;
import com.team1678.lib.requests.Request;
import com.team254.lib.drivers.TalonUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeRollers extends Subsystem {
	private static IntakeRollers mInstance;

	public static IntakeRollers getInstance() {
		if (mInstance == null) {
			mInstance = new IntakeRollers();
		}
		return mInstance;
	}

	public enum State {
		IDLE(0.0),
		INTAKING(8.0),
		EXHAUST(-6.0);

		public double roller_voltage;

		State(double roller_voltage) {
			this.roller_voltage = roller_voltage;
		}
	}

	private final TalonFX mRoller;

	private State mState = State.IDLE;
	private PeriodicIO mPeriodicIO = new PeriodicIO();

	private IntakeRollers() {
		mRoller = new TalonFX(Ports.INTAKE_ROLLER.getDeviceNumber(), Ports.INTAKE_ROLLER.getBus());
		TalonUtil.applyAndCheckConfiguration(mRoller, IntakeRollerConstants.RollerFXConfig());
		mRoller.setInverted(false);
	}

	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {}

			@Override
			public void onLoop(double timestamp) {
				mPeriodicIO.roller_demand = mState.roller_voltage;
			}

			@Override
			public void onStop(double timestamp) {}
		});
	}

	private static class PeriodicIO implements Sendable {
		// Inputs
		private double roller_output_voltage;
		private double roller_stator_current;
		private double roller_velocity;

		// Outputs
		private double roller_demand;

		@Override
		public void initSendable(SendableBuilder builder) {
			builder.addDoubleProperty("Demand", () -> roller_demand, null);
			builder.addDoubleProperty("VelocityRpS", () -> roller_velocity, null);
			builder.addDoubleProperty("OutputVoltage", () -> roller_output_voltage, null);
			builder.addDoubleProperty("StatorCurrent", () -> roller_stator_current, null);
		}
	}

	/**
	 * Gets the current state of the intake rollers.
	 *
	 * @return The current state.
	 */
	public State getState() {
		return mState;
	}

	/**
	 * Sets the state of the intake rollers.
	 *
	 * @param state The state to set.
	 */
	public void setState(State state) {
		mState = state;
	}

	/**
	 * @param _wantedState Wanted state for the intake rollers.
	 * @return New request that updates the intake rollers with the wanted state. 
	 */
	public Request stateRequest(State _wantedState) {
		return new Request() {
			@Override
			public void act() {
				setState(_wantedState);
			}

			@Override
			public boolean isFinished() {
				return mPeriodicIO.roller_demand == _wantedState.roller_voltage;
			}
		};
	}

	@Override
	public void readPeriodicInputs() {
		mPeriodicIO.roller_output_voltage = mRoller.getMotorVoltage().getValue();
		mPeriodicIO.roller_stator_current = mRoller.getStatorCurrent().getValue();
		mPeriodicIO.roller_velocity = mRoller.getVelocity().getValue();
	}

	@Override
	public void writePeriodicOutputs() {
		mRoller.setControl(new VoltageOut(mPeriodicIO.roller_demand));
	}

	@Override
	public void stop() {
		mPeriodicIO.roller_demand = 0.0;
	}

	@Override
	public boolean checkSystem() {
		return false;
	}

	@Override
	public void outputTelemetry() {
		SmartDashboard.putString("IntakeRollers/State", mState.toString());
		SmartDashboard.putData("IntakeRollers/IO", mPeriodicIO);
	}
}
