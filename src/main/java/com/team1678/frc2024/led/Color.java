package com.team1678.frc2024.led;

public class Color {

	public static Color RED = new Color(255, 0, 0);
	public static Color RED_DIMMED = new Color(120, 0, 0);
	public static Color PINK = new Color(255, 18, 143);
	public static Color GREEN = new Color(0, 255, 8);
	public static Color GREEN_DIMMED = new Color(0, 50, 0);
	public static Color PURPLE = new Color(196, 18, 255);
	public static Color ORANGE = new Color(255, 53, 13);
	public static Color YELLOW = new Color(255, 150, 5);
	public static Color CYAN = new Color(52, 155, 235);
	public static Color BLUE = new Color(0, 0, 255);
	public static Color BLUE_DIMMED = new Color(0, 0, 120);

	public int r;
	public int g;
	public int b;

	public Color(int red, int green, int blue) {
		r = red;
		g = green;
		b = blue;
	}

	public static Color off() {
		return new Color(0, 0, 0);
	}

	@Override
	public String toString() {
		return "(" + r + "," + g + "," + b + ")";
	}
}
