package com.team1678.frc2024.shooting;

public class FerryRegression {
	public static double[][] kLowFerryAngle = {
		// @x --> distance from target (meters)
		// @y --> hood angle (degrees)
		{5.3, 40.0},
		{7.9, 45.0},
		{9.3, 45.0},
		{12.3, 45.0},
	};

	public static double[][] kLowFerryRPM = {
		// @x --> distance from target (meters)
		// @y --> shooter speed (rpm)
		{5.3, 2000.0},
		{7.9, 3000.0},
		{9.3, 3500.0},
		{12.3, 4000.0},
	};

	public static double[][] kHighFerryAngle = {
		// @x --> distance from target (meters)
		// @y --> hood angle (degrees)
		{5.3, 53.0},
		{7.9, 53.0},
		{9.3, 53.0},
		{12.3, 53.0},
	};

	public static double[][] kHighFerryRPM = {
		// @x --> distance from target (meters)
		// @y --> shooter speed (rpm)
		{5.3, 2700.0},
		{7.9, 3700.0},
		{9.3, 4200.0},
		{12.3, 4700.0},
	};
}
