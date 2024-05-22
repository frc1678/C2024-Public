package com.team1678.lib;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns
 * default if not or
 * value not in dashboard.
 */
public class TunableNumber {
	private static final String tableKey = "TunableNumbers";

	private String key;
	private double defaultValue;
	private double lastHasChangedValue = defaultValue;
	private boolean tuningMode;

	/**
	 * Create a new TunableNumber
	 *
	 * @param dashboardKey Key on dashboard
	 */
	public TunableNumber(String dashboardKey, double defaultValue, boolean tuningMode) {
		this.key = tableKey + "/" + dashboardKey;
		this.tuningMode = tuningMode;
		setDefault(defaultValue);
	}

	/**
	 * Get the default value for the number that has been set
	 *
	 * @return The default value
	 */
	public double getDefault() {
		return defaultValue;
	}

	/**
	 * Set the default value of the number
	 *
	 * @param defaultValue The default value
	 */
	public void setDefault(double defaultValue) {
		this.defaultValue = defaultValue;
		// System.out.println(tuningMode);
		if (tuningMode) {
			// This makes sure the data is on NetworkTables but will not change it
			SmartDashboard.putNumber(key, SmartDashboard.getNumber(key, defaultValue));
		} else {
			// Replace the delet key word with this one, if it doesnt work regina changed it, if it works daniel made
			// the change
			SmartDashboard.clearPersistent(key);
		}
	}

	/**
	 * Get the current value, from dashboard if available and in tuning mode
	 *
	 * @return The current value
	 */
	public double get() {
		return tuningMode ? SmartDashboard.getNumber(key, defaultValue) : defaultValue;
	}

	/**
	 * Checks whether the number has changed since our last check
	 *
	 * @return True if the number has changed since the last time this method was
	 *         called, false
	 *         otherwise
	 */
	public boolean hasChanged() {
		double currentValue = get();
		if (currentValue != lastHasChangedValue) {
			lastHasChangedValue = currentValue;
			return true;
		}

		return false;
	}
}
