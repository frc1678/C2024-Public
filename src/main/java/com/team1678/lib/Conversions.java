package com.team1678.lib;

public class Conversions {

	/**
	 * @param counts Falcon Counts
	 * @param gearRatio Gear Ratio between Falcon and Mechanism
	 * @return Degrees of Rotation of Mechanism
	 */
	public static double falconToDegrees(double counts, double gearRatio) {
		return counts * (360.0 / (gearRatio * 2048.0));
	}

	/**
	 * @param degrees Degrees of rotation of Mechanism
	 * @param gearRatio Gear Ratio between Falcon and Mechanism
	 * @return Falcon Counts
	 */
	public static double degreesToFalcon(double degrees, double gearRatio) {
		double ticks = degrees / (360.0 / (gearRatio * 2048.0));
		return ticks;
	}

	/**
	 * @param counts    Falcon Counts
	 * @param gearRatio Gear Ratio between Falcon and Mechanism
	 * @return Degrees of Rotation of Mechanism
	 */
	public static double rotationsToDegrees(double counts, double gearRatio) {
		return counts * (360.0 / (gearRatio));
	}

	/**
	 * @param degrees   Degrees of rotation of Mechanism
	 * @param gearRatio Gear Ratio between Falcon and Mechanism
	 * @return Falcon Counts
	 */
	public static double degreesToRotation(double degrees, double gearRatio) {
		double rot = degrees / (360.0 / (gearRatio));
		return rot;
	}

	/**
	 * @param velocityCounts Falcon Velocity Counts
	 * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
	 * @return RPM of Mechanism
	 */
	public static double falconToRPM(double velocityCounts, double gearRatio) {
		double motorRPM = velocityCounts * (600.0 / 2048.0);
		double mechRPM = motorRPM / gearRatio;
		return mechRPM;
	}

	/**
	 * @param RPM RPM of mechanism
	 * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
	 * @return RPM of Mechanism
	 */
	public static double RPMToFalcon(double RPM, double gearRatio) {
		double motorRPM = RPM * gearRatio;
		double sensorCounts = motorRPM * (2048.0 / 600.0);
		return sensorCounts;
	}

	/**
	 * @param velocitycounts Falcon Velocity Counts
	 * @param circumference Circumference of Wheel
	 * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
	 * @return Falcon Velocity Counts
	 */
	public static double falconToMPS(double velocitycounts, double circumference, double gearRatio) {
		double wheelRPM = falconToRPM(velocitycounts, gearRatio);
		double wheelMPS = (wheelRPM * circumference) / 60;
		return wheelMPS;
	}

	/**
	 * @param velocity Velocity MPS
	 * @param circumference Circumference of Wheel
	 * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
	 * @return Falcon Velocity Counts
	 */
	public static double MPSToFalcon(double velocity, double circumference, double gearRatio) {
		double wheelRPM = ((velocity * 60) / circumference);
		double wheelVelocity = RPMToFalcon(wheelRPM, gearRatio);
		return wheelVelocity;
	}

	public static double MPSToRPS(double velocity, double circumference, double gearRatio) {
		double wheelRPS = ((velocity) / circumference);
		double falconRPS = wheelRPS * gearRatio;
		return falconRPS;
	}

	public static double RPSToMPS(double rotationsPerSecond, double circumference, double gearRatio) {
		double wheelRPM = (rotationsPerSecond / gearRatio);
		double mps = wheelRPM * circumference;
		return mps;
	}

	public static double falconToMeters(double falconTicks, double circumference, double gearRatio) {
		double wheelRevolutions = falconToDegrees(falconTicks, gearRatio) / 360.0;
		double wheelDistance = wheelRevolutions * circumference;
		return wheelDistance;
	}

	public static double rotationsToMeters(double rotations, double circumference, double gearRatio) {
		double wheelRevolutions = rotations / gearRatio;
		double wheelDistance = wheelRevolutions * circumference;
		return wheelDistance;
	}

	// Convert meters to inches
	public static double metersToInches(double meters) {
		return meters * (39.73701 / 1);
	}

	// Convert meters to inches
	public static double inchesToMeters(double inches) {
		return inches * (0.0254 / 1);
	}
}
