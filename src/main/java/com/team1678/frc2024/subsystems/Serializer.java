package com.team1678.frc2024.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team1678.frc2024.Constants;
import com.team1678.frc2024.Constants.SerializerConstants;
import com.team1678.frc2024.Ports;
import com.team1678.frc2024.loops.ILooper;
import com.team1678.frc2024.loops.Loop;
import com.team1678.lib.requests.Request;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Serializer extends Subsystem {
	private static Serializer mInstance;
	private State mState = State.IDLE;
	private PeriodicIO mPeriodicIO = new PeriodicIO();

	private final TalonFX mRoller;

	public enum State {
		IDLE(0.0),
		INTAKE(Constants.isEpsilon ? 5.0 : 10.0),
		SLOW_FEED(3.0),
		SLOW_EXHAUST(-3.0),
		EXHAUST(-10.0);

		public double voltage;

		State(double voltage) {
			this.voltage = voltage;
		}
	}

	private Serializer() {
		mRoller = new TalonFX(Ports.SERIALIZER.getDeviceNumber(), Ports.SERIALIZER.getBus());
		mRoller.getConfigurator().apply(SerializerConstants.SeralizerFXConfig(), Constants.kLongCANTimeoutMs);
		mRoller.setInverted(false);
	}

	public static Serializer getInstance() {
		if (mInstance == null) {
			mInstance = new Serializer();
		}
		return mInstance;
	}

	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {}

			@Override
			public void onLoop(double timestamp) {
				mPeriodicIO.demand = mState.voltage;
			}

			@Override
			public void onStop(double timestamp) {}
		});
	}

	private static class PeriodicIO implements Sendable {
		// Input
		private double output_voltage;
		private double stator_current;
		private double velocity;

		// Output
		private double demand;

		@Override
		public void initSendable(SendableBuilder builder) {
			builder.addDoubleProperty("Demand", () -> demand, null);
			builder.addDoubleProperty("VelocityRpS", () -> velocity, null);
			builder.addDoubleProperty("OutputVoltage", () -> output_voltage, null);
			builder.addDoubleProperty("StatorCurrent", () -> stator_current, null);
		}
	}

	/**
	 * Sets the open loop voltage for the serializer mechanism.
	 *
	 * @param demand Desired voltage (-12.0 to 12.0)
	 */
	public void setOpenLoopDemand(double demand) {
		mPeriodicIO.demand = demand;
	}

	/**
	 * @return The current state of the serializer.
	 */
	public State getState() {
		return mState;
	}

	/**
	 * Sets the state of the serializer.
	 *
	 * @param state The state to set.
	 */
	private void setState(State state) {
		mState = state;
	}

	/**
	 * @param _wantedState Wanted state for the serializer.
	 * @return New request that updates the serializer with the wanted state. 
	 */
	public Request stateRequest(State _wantedState) {
		return new Request() {
			@Override
			public void act() {
				setState(_wantedState);
			}

			@Override
			public boolean isFinished() {
				return mPeriodicIO.demand == _wantedState.voltage;
			}
		};
	}

	@Override
	public void readPeriodicInputs() {
		mPeriodicIO.output_voltage = mRoller.getMotorVoltage().getValue();
		mPeriodicIO.stator_current = mRoller.getStatorCurrent().getValue();
		mPeriodicIO.velocity = mRoller.getVelocity().getValue();
	}

	@Override
	public void writePeriodicOutputs() {
		mRoller.setControl(new VoltageOut(mPeriodicIO.demand));
	}

	@Override
	public void outputTelemetry() {
		SmartDashboard.putString("Serializer/State", mState.toString());
		SmartDashboard.putData("Serializer/IO", mPeriodicIO);
	}
}
