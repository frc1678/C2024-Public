package com.team1678.frc2024.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdleControlFrame;
import com.ctre.phoenix.led.CANdleStatusFrame;
import com.team1678.frc2024.Constants;
import com.team1678.frc2024.Ports;
import com.team1678.frc2024.Robot;
import com.team1678.frc2024.led.Color;
import com.team1678.frc2024.led.TimedLEDState;
import com.team1678.frc2024.loops.ILooper;
import com.team1678.frc2024.loops.Loop;
import com.team1678.frc2024.subsystems.vision.VisionDeviceManager;
import com.team1678.lib.requests.Request;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LEDs extends Subsystem {
	private static LEDs mInstance;

	public static LEDs getInstance() {
		if (mInstance == null) {
			mInstance = new LEDs();
		}
		return mInstance;
	}

	private final int kNumLeds = 8 + 18;

	private boolean mDisabled = true;
	private final CANdle mCandle = new CANdle(Ports.LEDS.getDeviceNumber(), Ports.LEDS.getBus());
	private LEDSection mLEDStatus = new LEDSection(0, kNumLeds);

	public LEDs() {
		CANdleConfiguration configAll = new CANdleConfiguration();
		configAll.statusLedOffWhenActive = false;
		configAll.disableWhenLOS = true;
		configAll.stripType = LEDStripType.RGB;
		configAll.brightnessScalar = 1.0;
		configAll.vBatOutputMode = VBatOutputMode.Modulated;
		mCandle.configAllSettings(configAll, Constants.kLongCANTimeoutMs);
		mCandle.setStatusFramePeriod(CANdleStatusFrame.CANdleStatusFrame_Status_1_General, 255);
		mCandle.setControlFramePeriod(CANdleControlFrame.CANdle_Control_1_General, 10);
		mCandle.setControlFramePeriod(CANdleControlFrame.CANdle_Control_2_ModulatedVBatOut, 255);
		applyStates(TimedLEDState.DISABLE_BLUE);
	}

	@Override
	public void registerEnabledLoops(ILooper mEnabledLooper) {
		mEnabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {
				mDisabled = false;
				applyStates(TimedLEDState.OFF);
			}

			@Override
			public void onLoop(double timestamp) {}

			@Override
			public void onStop(double timestamp) {
				mDisabled = true;
				mLEDStatus.reset();
				applyStates(TimedLEDState.DISABLE_BLUE);
			}
		});
	}

	@Override
	public void readPeriodicInputs() {
		if (mDisabled) {
			if (!VisionDeviceManager.getInstance().fullyConnected()) {
				applyStates(TimedLEDState.NO_VISION);
			} else {
				if (Robot.is_red_alliance) {
					applyStates(TimedLEDState.DISABLE_RED);
				} else {
					applyStates(TimedLEDState.DISABLE_BLUE);
				}
			}
		}

		double timestamp = Timer.getFPGATimestamp();
		if (mLEDStatus.state.interval != Double.POSITIVE_INFINITY) {
			if (timestamp - mLEDStatus.lastSwitchTime >= mLEDStatus.state.interval) {
				mLEDStatus.nextColor();
				mLEDStatus.lastSwitchTime = timestamp;
			}
		}

		Color color = mLEDStatus.getWantedColor();
		mCandle.setLEDs(color.r, color.g, color.b, 0, mLEDStatus.startIDx, 100);
	}

	// setter functions
	public void applyStates(TimedLEDState TimedState) {
		mLEDStatus.setState(TimedState);
	}

	@Override
	public void stop() {}

	@Override
	public boolean checkSystem() {
		return false;
	}

	@Override
	public void outputTelemetry() {
		SmartDashboard.putString("LED Status", mLEDStatus.state.name);
		SmartDashboard.putString("LED Colors", mLEDStatus.getWantedColor().toString());
	}

	/**
	 * Updates the LEDs to a specific color or animation.
	 *
	 * @param wanted_state Wanted LED color/animation.
	 */
	public Request stateRequest(TimedLEDState wanted_state) {
		return new Request() {

			@Override
			public void act() {
				applyStates(wanted_state);
			}

			@Override
			public boolean isFinished() {
				return true;
			}
		};
	}

	// Class for holding information about each section
	private class LEDSection {
		private TimedLEDState state = TimedLEDState.OFF; // current TimedState
		private double lastSwitchTime = 0.0; // timestampe of last color cycle
		private int colorIndex = 0; // tracks current color in array
		private int startIDx, LEDCount; // start and end of section

		public LEDSection(int startIndex, int endIndex) {
			startIDx = startIndex;
			LEDCount = endIndex - startIndex;
		}

		public void setState(TimedLEDState wantedTimedState) {
			if (wantedTimedState != state) {
				colorIndex = 0;
				lastSwitchTime = Timer.getFPGATimestamp();
				state = wantedTimedState;
			}
		}

		public Color getWantedColor() {
			Color color;
			try {
				color = state.colors[colorIndex];
			} catch (Exception e) {
				color = Color.off();
			}
			return color;
		}

		// cycle to next color in array
		public void nextColor() {
			if (state.colors.length == 1) {
				return;
			}
			if (colorIndex == state.colors.length - 1) {
				colorIndex = 0;
			} else {
				colorIndex++;
			}
		}

		public void reset() {
			state = TimedLEDState.OFF;
			lastSwitchTime = 0.0;
			colorIndex = 0;
		}
	}
}
