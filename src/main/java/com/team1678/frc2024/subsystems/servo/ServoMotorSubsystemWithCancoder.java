package com.team1678.frc2024.subsystems.servo;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.team1678.frc2024.Constants;
import com.team254.lib.drivers.CanDeviceId;
import com.team254.lib.drivers.Phoenix6Util;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ServoMotorSubsystemWithCancoder extends ServoMotorSubsystem {

	public static class AbsoluteEncoderConstants {
		public FeedbackSensorSourceValue encoder_type;
		public CanDeviceId remote_encoder_port;
		public double rotor_rotations_per_output;
		public double remote_encoder_offset;
	}

	private CANcoder mCancoder;
	private AbsoluteEncoderConstants mEncoderConstants;

	protected ServoMotorSubsystemWithCancoder(
			ServoMotorSubsystemConstants constants, AbsoluteEncoderConstants encoder_constants) {
		super(constants);
		mEncoderConstants = encoder_constants;

		mCancoder = new CANcoder(
				encoder_constants.remote_encoder_port.getDeviceNumber(),
				encoder_constants.remote_encoder_port.getBus());
		CANcoderConfiguration cancoder_configuration = new CANcoderConfiguration();
		cancoder_configuration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
		cancoder_configuration.MagnetSensor.MagnetOffset = encoder_constants.remote_encoder_offset;
		Phoenix6Util.checkErrorAndRetry(() -> mCancoder.getConfigurator().apply(cancoder_configuration));
		System.out.println(encoder_constants.remote_encoder_offset);
		mCancoder.getAbsolutePosition().waitForUpdate(Constants.kLongCANTimeoutMs);
		double position = mCancoder.getAbsolutePosition().getValueAsDouble();
		while (position < 0.0) {
			position++;
		}
		while (position >= 0.9) {
			position--;
		}
		final double set_position = position;
		Phoenix6Util.checkErrorAndRetry(() -> mCancoder.setPosition(set_position));
		SmartDashboard.putNumber("Cancoder abs", set_position);

		changeTalonConfig((conf) -> {
			conf.Feedback.FeedbackSensorSource = mEncoderConstants.encoder_type;
			conf.Feedback.FeedbackRemoteSensorID = mEncoderConstants.remote_encoder_port.getDeviceNumber();
			conf.Feedback.RotorToSensorRatio =
					(encoder_constants.rotor_rotations_per_output) / (mConstants.kRotationsPerUnitDistance * 360.0);
			conf.Feedback.SensorToMechanismRatio = 1.0;
			return conf;
		});
	}

	@Override
	public synchronized void zeroSensors() {
		return;
	}

	public synchronized void setPosition(double value) {
		mCancoder.setPosition(value);
	}

	private double rotationsToHomedRotations(double input) {
		double rot = input + mEncoderConstants.remote_encoder_offset;
		while (rot > 1.0) {
			rot--;
		}
		while (rot < 0.0) {
			rot++;
		}
		return rot;
	}
}
