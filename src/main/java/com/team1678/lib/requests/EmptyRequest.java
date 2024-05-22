package com.team1678.lib.requests;

public class EmptyRequest extends Request {

	@Override
	public void act() {
		// empty, as the name suggests
	}

	@Override
	public boolean isFinished() {
		return true;
	}

	@Override
	public String toString() {
		return "EmptyRequest()";
	}
}
