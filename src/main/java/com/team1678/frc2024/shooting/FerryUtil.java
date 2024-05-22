package com.team1678.frc2024.shooting;

import com.team1678.frc2024.FieldLayout;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.Util;

public class FerryUtil {

	private static final double kOppoWingToAllianceWall =
			FieldLayout.distanceFromAllianceWall(FieldLayout.kWingX, true);
	public static final Translation2d kCornerTarget = new Translation2d(1.0, FieldLayout.kFieldWidth - 1.5);
	public static final Translation2d kMidTarget =
			new Translation2d((FieldLayout.kFieldLength / 2.0) - 1.0, FieldLayout.kFieldWidth - 0.2);

	/**
	 * Get adaptive ferry shot parameters
	 * @param robot_pose Pose of the robot
	 * @param is_red_alliance
	 * @return Array of length 4 containing distance to target, hood angle, shooter rpm, and drivetrain angle
	 */
	public static double[] getFerryShotParameters(Pose2d robot_pose, boolean is_red_alliance) {
		boolean midfield_target = useMidfieldTarget(robot_pose.getTranslation().x(), is_red_alliance);
		Translation2d target;
		if (midfield_target) {
			target = FieldLayout.handleAllianceFlip(kMidTarget, is_red_alliance);
		} else {
			target = FieldLayout.handleAllianceFlip(kCornerTarget, is_red_alliance);
		}
		Translation2d robot_to_target =
				target.translateBy(robot_pose.getTranslation().inverse());
		Rotation2d target_drive_heading = robot_to_target
				.direction()
				.rotateBy(Rotation2d.fromDegrees(180.0))
				.rotateBy(Rotation2d.fromDegrees(-10.0));
		double shooter_setpoint, hood_setpoint, dist_to_target;
		dist_to_target = robot_to_target.norm();
		if (inHighFerryZone(robot_pose, is_red_alliance) || midfield_target) {
			hood_setpoint =
					RegressionMaps.kHighFerryHoodMap.getInterpolated(new InterpolatingDouble(dist_to_target)).value;
			shooter_setpoint =
					RegressionMaps.kHighFerryRPMMap.getInterpolated(new InterpolatingDouble(dist_to_target)).value;
			target_drive_heading = target_drive_heading.rotateBy(Rotation2d.fromDegrees(0.0));
		} else {
			hood_setpoint =
					RegressionMaps.kLowFerryHoodMap.getInterpolated(new InterpolatingDouble(dist_to_target)).value;
			shooter_setpoint =
					RegressionMaps.kLowFerryRPMMap.getInterpolated(new InterpolatingDouble(dist_to_target)).value;
		}
		return new double[] {dist_to_target, hood_setpoint, shooter_setpoint, target_drive_heading.getDegrees()};
	}

	private static boolean inHighFerryZone(Pose2d robot_pose, boolean is_red_alliance) {
		double x = robot_pose.getTranslation().x();
		double y = robot_pose.getTranslation().y();
		Translation2d cor_0 =
				FieldLayout.handleAllianceFlip(new Translation2d(FieldLayout.kWingX, 0.0), is_red_alliance);
		Translation2d cor_1 = FieldLayout.kCenterNote2;
		boolean in_x = Util.inRange(x, Math.min(cor_0.x(), cor_1.x()), Math.max(cor_0.x(), cor_1.x()));
		boolean in_y = Util.inRange(y, Math.min(cor_0.y(), cor_1.y()), Math.max(cor_0.y(), cor_1.y()));
		return in_x && in_y;
	}

	private static boolean useMidfieldTarget(double x_coord, boolean is_red_alliance) {
		return FieldLayout.distanceFromAllianceWall(x_coord, is_red_alliance) - kOppoWingToAllianceWall > 1.0;
	}
}
