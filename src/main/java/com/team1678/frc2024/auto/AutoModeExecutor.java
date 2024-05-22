package com.team1678.frc2024.auto;

import com.team1678.frc2024.loops.CrashTrackingRunnable;

/**
 * This class selects, runs, and stops (if necessary) a specified autonomous
 * mode.
 */
public class AutoModeExecutor {
	private AutoModeBase m_auto_mode;
	private Thread m_thread = null;

	public void setAutoMode(AutoModeBase new_auto_mode) {
		m_auto_mode = new_auto_mode;
	}

	public AutoModeBase getAutoMode() {
		return m_auto_mode;
	}

	public void start() {
		if (m_thread == null) {
			m_thread = new Thread(new CrashTrackingRunnable() {
				@Override
				public void runCrashTracked() {
					if (m_auto_mode != null) {
						m_auto_mode.run();
					}
				}
			});

			m_thread.start();
		}
	}

	public void stop() {
		if (m_auto_mode != null) {
			m_auto_mode.stop();
		}

		m_thread = null;
	}
}
