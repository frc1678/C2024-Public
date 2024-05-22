package com.team1678.frc2024.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team1678.frc2024.Constants;
import com.team1678.frc2024.Ports;
import com.team1678.frc2024.loops.ILooper;
import com.team1678.frc2024.loops.Loop;
import com.team1678.lib.requests.Request;
import com.team254.lib.drivers.TalonUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Feeder extends Subsystem {
	private static Feeder mInstance;

	public static synchronized Feeder getInstance() {
		if (mInstance == null) {
			mInstance = new Feeder();
		}
		return mInstance;
	}

	public State mState = State.IDLE;
	private PeriodicIO mPeriodicIO = new PeriodicIO();

	private static TalonFX mFeeder;

	public Feeder() {
		mFeeder = new TalonFX(Ports.FEEDER.getDeviceNumber(), Ports.FEEDER.getBus());
		mFeeder.setInverted(true);
		mFeeder.setNeutralMode(NeutralModeValue.Brake);
		TalonUtil.applyAndCheckConfiguration(mFeeder, Constants.FeederConstants.FeederFXConfig());
	}

	public enum State {
		IDLE(0.0),
		SHOOTER_FEED(6000.0),
		SLOW_FEED(4000.0),
		DE_AMP(2000.0),
		AMP_FEED(-5400.0);

		public double wheel_rpm;

		State(double wheel_rpm) {
			this.wheel_rpm = wheel_rpm;
		}
	}

	public State getState() {
		return mState;
	}

	private void setState(State state) {
		mState = state;
	}

	public static class PeriodicIO implements Sendable {

		// Inputs
		private double voltage;
		private double current;
		private double velocity;

		// Outputs
		private double demand;

		@Override
		public void initSendable(SendableBuilder builder) {
			builder.addDoubleProperty("Demand", () -> demand, null);
			builder.addDoubleProperty("VelocityRpS", () -> velocity, null);
			builder.addDoubleProperty("OutputVoltage", () -> voltage, null);
			builder.addDoubleProperty("StatorCurrent", () -> current, null);
		}
	}

	@Override
	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {}

			@Override
			public void onLoop(double timestamp) {
				switch (mState) {
					case IDLE:
					case SHOOTER_FEED:
					case AMP_FEED:
					case DE_AMP:
					case SLOW_FEED:
						mPeriodicIO.demand = mState.wheel_rpm;
						break;
				}
			}

			@Override
			public void onStop(double timestamp) {}
		});
	}

	@Override
	public void writePeriodicOutputs() {
		mFeeder.setControl(
				new VelocityVoltage(mPeriodicIO.demand / Constants.FeederConstants.kFeederVelocityConversion));
	}

	@Override
	public void readPeriodicInputs() {
		mPeriodicIO.velocity = mFeeder.getVelocity().getValue() * Constants.FeederConstants.kFeederVelocityConversion;
		mPeriodicIO.current = mFeeder.getStatorCurrent().getValue();
		mPeriodicIO.voltage = mFeeder.getMotorVoltage().getValue();
	}

	@Override
	public void outputTelemetry() {
		SmartDashboard.putString("Feeder/Roller State", mState.toString());
		SmartDashboard.putData("Feeder/IO", mPeriodicIO);
	}

	/**
	 * Returns new request that updates the state of the Feeder.
	 *
	 * @param _wantedState New desired state.
	 */
	public Request stateRequest(State wantedstate) {
		return new Request() {

			@Override
			public void act() {
				setState(wantedstate);
			}

			@Override
			public boolean isFinished() {
				return mPeriodicIO.demand == wantedstate.wheel_rpm;
			}
		};
	}
}
