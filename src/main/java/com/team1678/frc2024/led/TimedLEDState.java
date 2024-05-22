package com.team1678.frc2024.led;

public class TimedLEDState {
	private static final double kFlashInterval = 0.2;

	// spotless:off
	
	public static final TimedLEDState OFF = new TimedLEDState("OFF", Double.POSITIVE_INFINITY, Color.off());
	public static final TimedLEDState DISABLE_BLUE = new TimedLEDState("DISABLE_BLUE", 1.0, Color.GREEN_DIMMED, Color.BLUE_DIMMED);
	public static final TimedLEDState DISABLE_RED = new TimedLEDState("DISABLE_RED", 1.0, Color.GREEN_DIMMED, Color.RED_DIMMED);
	public static final TimedLEDState NO_VISION = new TimedLEDState("NO_VISION", Double.POSITIVE_INFINITY, Color.ORANGE);
	public static final TimedLEDState EMERGENCY = new TimedLEDState("EMERGENCY", kFlashInterval, Color.RED, Color.off());
	
	public static final TimedLEDState NOTE_HELD_SHOT = new TimedLEDState("NOTE_HELD_SHOT", Double.POSITIVE_INFINITY, Color.GREEN);
	public static final TimedLEDState NOTE_HELD_FERRY =	new TimedLEDState("NOTE_HELD_FERRY", Double.POSITIVE_INFINITY, Color.YELLOW);

	public static final TimedLEDState ELEVATOR_LOADING = new TimedLEDState("ELEVATOR_LOADING", kFlashInterval, Color.RED, Color.off());
	public static final TimedLEDState ELEVATOR_LOADED = new TimedLEDState("ELEVATOR_LOADED", Double.POSITIVE_INFINITY, Color.BLUE);
	public static final TimedLEDState FIRING = new TimedLEDState("FIRING", Double.POSITIVE_INFINITY, Color.ORANGE);

	public static final TimedLEDState CONTINUOUS_SHOT = new TimedLEDState("CONTINUOUS_SHOT", 0.2, Color.ORANGE, Color.off());

	// spotless:on

	public Color[] colors; // array of colors to iterate over
	public double interval; // time in seconds between states
	public String name; // name of state

	public TimedLEDState(String name, double interval, Color... colors) {
		this.colors = colors;
		this.interval = interval;
		this.name = name;
	}
}
