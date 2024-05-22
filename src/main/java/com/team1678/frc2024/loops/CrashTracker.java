package com.team1678.frc2024.loops;

import java.util.UUID;

// CREDIT - FRC Team 1323

/**
 * Tracks start-up and caught crash events, logging them to a file which dosn't roll over
 */
public class CrashTracker {

	private static final UUID RUN_INSTANCE_UUID = UUID.randomUUID();

	public static void logRobotStartup() {
		logMarker("robot startup");
	}

	public static void logRobotConstruction() {
		logMarker("robot startup");
	}

	public static void logRobotInit() {
		logMarker("robot init");
	}

	public static void logTeleopInit() {
		logMarker("teleop init");
	}

	public static void logAutoInit() {
		logMarker("auto init");
	}

	public static void logDisabledInit() {
		logMarker("disabled init");
	}

	public static void logThrowableCrash(Throwable throwable) {
		logMarker("Exception", throwable);
	}

	private static void logMarker(String mark) {
		logMarker(mark, null);
	}

	private static void logMarker(String mark, Throwable nullableException) {}
}
