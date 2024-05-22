package com.team1678.frc2024.subsystems;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team1678.frc2024.Constants;
import com.team1678.frc2024.loops.ILooper;
import com.team1678.frc2024.loops.Loop;
import com.team1678.frc2024.subsystems.servo.ServoMotorSubsystem;

public class Climber extends ServoMotorSubsystem {
	private static Climber mInstance;

	public static Climber getInstance() {
		if (mInstance == null) {
			mInstance = new Climber();
		}
		return mInstance;
	}

	public static final double kExtensionHeight = 0.487;
	public static final double kPrepHeight = 0.306;
	public static final double kPullHeight = 0.02;
	public static final double kRetractionHeight = 0.0;

	private Climber() {
		super(Constants.ClimberConstants.kClimberServoConstants);
		zeroSensors();
	}

	@Override
	public void registerEnabledLoops(ILooper mEnabledLooper) {
		mEnabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {
				setOpenLoop(0.0);
			}

			public void onLoop(double timestamp) {}

			@Override
			public void onStop(double timestamp) {
				setNeutralMode(NeutralModeValue.Brake);
			}
		});
	}
}
