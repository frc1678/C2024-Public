package com.team1678.lib;

import com.team254.lib.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import java.util.List;

/**
 * Contains basic functions that are used often.
 */
public class Util {

	public static final double kEpsilon = 1e-12;

	/**
	 * Prevent this class from being instantiated.
	 */
	private Util() {}

	/**
	 * Limits the given input to the given magnitude.
	 */
	public static double limit(double v, double maxMagnitude) {
		return limit(v, -maxMagnitude, maxMagnitude);
	}

	public static double limit(double v, double min, double max) {
		return Math.min(max, Math.max(min, v));
	}

	public static boolean inRange(double v, double maxMagnitude) {
		return inRange(v, -maxMagnitude, maxMagnitude);
	}

	/**
	 * Checks if the given input is within the range (min, max), both exclusive.
	 */
	public static boolean inRange(double v, double min, double max) {
		return v > min && v < max;
	}

	public static double interpolate(double a, double b, double x) {
		x = limit(x, 0.0, 1.0);
		return a + (b - a) * x;
	}

	public static String joinStrings(final String delim, final List<?> strings) {
		StringBuilder sb = new StringBuilder();
		for (int i = 0; i < strings.size(); ++i) {
			sb.append(strings.get(i).toString());
			if (i < strings.size() - 1) {
				sb.append(delim);
			}
		}
		return sb.toString();
	}

	public static boolean epsilonEquals(double a, double b, double epsilon) {
		return (a - epsilon <= b) && (a + epsilon >= b);
	}

	public static boolean epsilonEquals(double a, double b) {
		return epsilonEquals(a, b, kEpsilon);
	}

	public static boolean epsilonEquals(int a, int b, int epsilon) {
		return (a - epsilon <= b) && (a + epsilon >= b);
	}

	public static boolean allCloseTo(final List<Double> list, double value, double epsilon) {
		boolean result = true;
		for (Double value_in : list) {
			result &= epsilonEquals(value_in, value, epsilon);
		}
		return result;
	}

	public static double clamp(double value, double min, double max) {
		if (min > max) {
			throw new IllegalArgumentException("min must not be greater than max");
		}

		return Math.max(min, Math.min(value, max));
	}

	public static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
		double lowerBound;
		double upperBound;
		double lowerOffset = scopeReference % 360;
		if (lowerOffset >= 0) {
			lowerBound = scopeReference - lowerOffset;
			upperBound = scopeReference + (360 - lowerOffset);
		} else {
			upperBound = scopeReference - lowerOffset;
			lowerBound = scopeReference - (360 + lowerOffset);
		}
		while (newAngle < lowerBound) {
			newAngle += 360;
		}
		while (newAngle > upperBound) {
			newAngle -= 360;
		}
		if (newAngle - scopeReference > 180) {
			newAngle -= 360;
		} else if (newAngle - scopeReference < -180) {
			newAngle += 360;
		}
		return newAngle;
	}

	public static double deadBand(double val, double deadband) {
		return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
	}

	public static double scaledDeadband(double value, double maxValue, double deadband) {
		double deadbandedValue = deadBand(value, deadband);
		if (epsilonEquals(deadbandedValue, 0.0)) return 0.0;
		return Math.signum(deadbandedValue) * ((Math.abs(deadbandedValue) - deadband) / (maxValue - deadband));
	}

	public static boolean shouldReverse(Rotation2d goalAngle, Rotation2d currentAngle) {
		double angleDifference = Math.abs(goalAngle.distance(currentAngle));
		double reverseAngleDifference =
				Math.abs(goalAngle.distance(currentAngle.rotateBy(Rotation2d.fromDegrees(180.0))));
		return reverseAngleDifference < angleDifference;
	}

	public static Rotation2d robotToFieldRelative(Rotation2d rot, boolean is_red_alliance) {
		if (is_red_alliance) {
			return rot.rotateBy(Rotation2d.fromDegrees(180.0));
		} else {
			return rot;
		}
	}

	public static double boundAngleNeg180to180Degrees(double angle) {
		// Naive algorithm
		while (angle >= 180.0) {
			angle -= 360.0;
		}
		while (angle < -180.0) {
			angle += 360.0;
		}
		return angle;
	}

	public static double boundAngle0to360Degrees(double angle) {
		// Naive algorithm
		while (angle >= 360.0) {
			angle -= 360.0;
		}
		while (angle < 0.0) {
			angle += 360.0;
		}
		return angle;
	}

	public static double boundToScope(double scopeFloor, double scopeCeiling, double argument) {
		double stepSize = scopeCeiling - scopeFloor;
		while (argument >= scopeCeiling) {
			argument -= stepSize;
		}
		while (argument < scopeFloor) {
			argument += stepSize;
		}
		return argument;
	}

	public static Pose2d toWPILibPose(com.team254.lib.geometry.Pose2d pose) {
		return new Pose2d(
				pose.getTranslation().x(),
				pose.getTranslation().y(),
				edu.wpi.first.math.geometry.Rotation2d.fromDegrees(
						pose.getRotation().getDegrees()));
	}

	public static com.team254.lib.geometry.Pose2d to254Pose(Pose2d pose) {
		return new com.team254.lib.geometry.Pose2d(
				pose.getTranslation().getX(),
				pose.getTranslation().getY(),
				Rotation2d.fromDegrees(pose.getRotation().getDegrees()));
	}

	public static edu.wpi.first.math.geometry.Rotation2d toWPILibRotation(Rotation2d rot) {
		return edu.wpi.first.math.geometry.Rotation2d.fromDegrees(rot.getDegrees());
	}

	public static double quaternionTo2dRadians(double w, double x, double y, double z) {
		return new Rotation3d(new Quaternion(w, x, y, z)).toRotation2d().getRadians();
	}
}
