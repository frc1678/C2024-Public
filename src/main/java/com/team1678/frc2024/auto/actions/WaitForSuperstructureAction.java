package com.team1678.frc2024.auto.actions;

import com.team1678.frc2024.subsystems.Superstructure;
import com.team1678.lib.util.Stopwatch;

public class WaitForSuperstructureAction implements Action {
	private Superstructure superstructure;
	private Stopwatch stopwatch;

	private double mTimeToWait;

	public WaitForSuperstructureAction(double timeToWait) {
		superstructure = Superstructure.getInstance();
		stopwatch = new Stopwatch();
		mTimeToWait = timeToWait;
	}

	@Override
	public boolean isFinished() {
		return stopwatch.getTime() >= mTimeToWait || superstructure.requestsCompleted();
	}

	@Override
	public void start() {
		stopwatch.start();
	}

	@Override
	public void update() {}

	@Override
	public void done() {}
}
