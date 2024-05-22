package com.team1678.lib.requests;

import java.util.function.BooleanSupplier;

public class IfRequest extends Request {
	private final BooleanSupplier condition;
	private final Request ifBranchRequest, elseBranchRequest;
	private Request activeRequest;
	private boolean hasRequestStarted = false;

	public IfRequest(BooleanSupplier condition, Request ifBranchRequest) {
		this(condition, ifBranchRequest, new EmptyRequest());
	}

	public IfRequest(BooleanSupplier condition, Request ifBranchRequest, Request elseBranchRequest) {
		this.condition = condition;
		this.ifBranchRequest = ifBranchRequest;
		this.elseBranchRequest = elseBranchRequest;
	}

	@Override
	public void cleanup() {
		if (activeRequest != null) {
			activeRequest.cleanup();
		}
		super.cleanup();
	}

	private void startRequestIfAllowed() {
		if (activeRequest.allowed()) {
			activeRequest.act();
			hasRequestStarted = true;
		}
	}

	@Override
	public void act() {
		// Choose a branch
		activeRequest = condition.getAsBoolean() ? ifBranchRequest : elseBranchRequest;

		startRequestIfAllowed();
	}

	@Override
	public boolean isFinished() {
		if (!hasRequestStarted) {
			startRequestIfAllowed();
		}

		if (hasRequestStarted) {
			return activeRequest.isFinished();
		}

		return false;
	}
}
