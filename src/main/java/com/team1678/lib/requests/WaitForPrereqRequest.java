package com.team1678.lib.requests;

import com.team1678.lib.util.Stopwatch;

public class WaitForPrereqRequest extends Request {
	private final Prerequisite prerequisite;
	private final double timeoutSeconds;
	private Stopwatch stopwatch;

	public WaitForPrereqRequest(Prerequisite prerequisite) {
		this(prerequisite, Double.POSITIVE_INFINITY);
	}

	public WaitForPrereqRequest(Prerequisite prerequisite, double timeoutSeconds) {
		this.prerequisite = prerequisite;
		this.timeoutSeconds = timeoutSeconds;
	}

	@Override
	public void act() {
		stopwatch.start();
	}

	@Override
	public boolean isFinished() {
		return prerequisite.met() || stopwatch.getTime() > timeoutSeconds;
	}
}
