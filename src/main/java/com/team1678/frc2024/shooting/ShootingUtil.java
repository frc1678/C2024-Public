package com.team1678.frc2024.shooting;

import com.team1678.frc2024.FieldLayout;
import com.team1678.frc2024.Robot;
import com.team1678.frc2024.subsystems.Superstructure;
import com.team1678.lib.logger.LogUtil;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.util.InterpolatingDouble;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShootingUtil {
	public static final boolean kEnableShootOnMove = true;
	public static final double kToFFactor = 0.2;

	private static final Pose2d kSpeakerTarget = FieldLayout.kSpeakerCenter;

	/**
	 * Get speaker shot parameters
	 * @param robot_pose Pose of the robot
	 * @param is_red_alliance
	 * @return Array of length 4 containing distance to target, hood angle, shooter rpm, and drivetrain angle
	 */
	public static double[] getSpeakerShotParameters(
			Pose2d robot_pose, Twist2d robot_velocity, boolean is_red_alliance) {
		Translation2d target = FieldLayout.handleAllianceFlip(kSpeakerTarget.getTranslation(), is_red_alliance);
		Translation2d robot_to_target =
				target.translateBy(robot_pose.getTranslation().inverse());

		LogUtil.recordTranslation2d(
				"LookaheadPose",
				robot_pose.getTranslation().translateBy(new Translation2d(robot_velocity.dx, robot_velocity.dy)));

		double yaw = robot_to_target.direction().getDegrees();
		double dist = robot_to_target.norm();
		double uncompensated_range = dist;

		SmartDashboard.putNumber("FiringParams/Uncomped Dist", dist);

		if (kEnableShootOnMove) {
			double[] sotf_params = getAdjustedShootOnMoveParams(yaw, dist, robot_velocity);
			yaw = sotf_params[0];
			dist = sotf_params[1];
		}

		double shooter_setpoint, hood_setpoint;
		Rotation2d target_drive_heading = Rotation2d.fromDegrees(yaw + 180.0);
		shooter_setpoint = getShooterSetpointForShot(dist);
		hood_setpoint = getHoodSetpointForShot(dist);
		SmartDashboard.putNumber("FiringParams/Y Dist", robot_to_target.y());

		target_drive_heading = target_drive_heading
				.rotateBy(Rotation2d.fromDegrees(getShooterCurveCompensationFromRegression(uncompensated_range)))
				.rotateBy(Rotation2d.fromDegrees(getSkewCompensationFromRegression(robot_to_target.y())));

		return new double[] {dist, hood_setpoint, shooter_setpoint, target_drive_heading.getDegrees()};
	}

	/**
	 * Get new parameters given robot velocity
	 * @param uncompensated_yaw
	 * @param uncompensated_range
	 * @param robot_velocity
	 * @return Array of length 2 containing adjusted yaw and adjusted range
	 */
	private static double[] getAdjustedShootOnMoveParams(
			double uncompensated_yaw, double uncompensated_range, Twist2d robot_velocity) {
		Translation2d polar_velocity = new Translation2d(robot_velocity.dx, robot_velocity.dy)
				.rotateBy(Rotation2d.fromDegrees(uncompensated_yaw));
		double radial = polar_velocity.x();
		double tangential = polar_velocity.y();

		SmartDashboard.putNumber("FiringParams/Tangential", tangential);
		SmartDashboard.putNumber("FiringParams/Radial", radial);

		double shot_speed = uncompensated_range / kToFFactor - radial;
		shot_speed = Math.max(0.0, shot_speed);
		double yaw_adj = Units.radiansToDegrees(Math.atan2(-tangential, shot_speed));
		double range_adj = kToFFactor * Math.sqrt(Math.pow(tangential, 2) + Math.pow(shot_speed, 2));
		return new double[] {yaw_adj + uncompensated_yaw, range_adj};
	}

	// interpolates distance to target for shooter setpoint along regression
	private static double getShooterSetpointForShot(double range) {
		// if (Superstructure.getInstance().kUseSmartdash) {
		//     return Superstructure.getInstance().mRPMTuner.get();
		// }
		return RegressionMaps.kFlywheelAutoAimMap.getInterpolated(new InterpolatingDouble(range)).value;
	}

	// interpolates distance to target for hood setpoint along regression
	private static double getHoodSetpointForShot(double range) {
		// if (Superstructure.getInstance().kUseSmartdash) {
		//     return Superstructure.getInstance().mHoodTuner.get();
		// }
		return RegressionMaps.kHoodAutoAimMap.getInterpolated(new InterpolatingDouble(range)).value;
	}

	// interpolates distance to target for compensation for shooter curve
	private static double getShooterCurveCompensationFromRegression(double range) {
		// if (Superstructure.getInstance().kUseSmartdash) {
		//     return Superstructure.getInstance().kCurveTuner.get();
		// }
		return RegressionMaps.kShooterCurveOffsetMap.getInterpolated(new InterpolatingDouble(range)).value;
	}

	// interpolates distance to target for hood setpoint along regression
	private static double getSkewCompensationFromRegression(double y_offset) {
		if (Superstructure.getInstance().kUseSmartdash) {
			return Superstructure.getInstance().kSkewTuner.get();
		}
		double offset = RegressionMaps.kSkewOffsetMap.getInterpolated(new InterpolatingDouble(y_offset)).value;
		return Robot.is_red_alliance ? offset : offset;
	}
}
