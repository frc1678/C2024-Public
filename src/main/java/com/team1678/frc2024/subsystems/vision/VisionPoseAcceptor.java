package com.team1678.frc2024.subsystems.vision;

import com.team1678.frc2024.FieldLayout;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Twist2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionPoseAcceptor {
	private static final double kFieldBorderMargin = 0.5;
	private static final double kMaxVisionCorrection = 2.0; // Jump from fused pose

	Pose2d mLastVisionFieldToVehicle = null;

	public boolean shouldAcceptVision(
			double timestamp,
			Pose2d visionFieldToVehicle,
			Pose2d lastFieldToVehicle,
			Twist2d robotVelocity,
			boolean isInAuto) {

		// If first update, trust
		if (mLastVisionFieldToVehicle == null) {
			mLastVisionFieldToVehicle = visionFieldToVehicle;
			return true;
		}

		// Write last pose early because we return out of the method
		mLastVisionFieldToVehicle = visionFieldToVehicle;

		// Check out of field
		if (visionFieldToVehicle.getTranslation().x() < -kFieldBorderMargin
				|| visionFieldToVehicle.getTranslation().x() > FieldLayout.kFieldLength + kFieldBorderMargin
				|| visionFieldToVehicle.getTranslation().y() < -kFieldBorderMargin
				|| visionFieldToVehicle.getTranslation().y() > FieldLayout.kFieldWidth + kFieldBorderMargin) {
			SmartDashboard.putString("Vision validation", "Outside field");
			return false;
		}

		if (robotVelocity.norm() > 4.0) {
			SmartDashboard.putString("Vision validation", "Max velocity");
			return false;
		}

		if (isInAuto) {
			// Check max correction
			if (visionFieldToVehicle.distance(lastFieldToVehicle) > kMaxVisionCorrection) {
				SmartDashboard.putString("Vision validation", "Max correction");
				return false;
			}
		}

		SmartDashboard.putString("Vision validation", "OK");
		return true;
	}
}
