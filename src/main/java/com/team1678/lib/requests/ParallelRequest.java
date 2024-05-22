/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1678.lib.requests;

import java.util.Arrays;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;

/**
 * A Request which takes a list of Requests and executes them in parallel.
 */
public class ParallelRequest extends Request {
	private final List<Request> idleRequests;
	private final List<Request> inProgressRequests;

	public ParallelRequest(Request... requests) {
		idleRequests = new LinkedList<>(Arrays.asList(requests));
		inProgressRequests = new LinkedList<>();
	}

	@Override
	public void cleanup() {
		inProgressRequests.forEach(r -> r.cleanup());
		idleRequests.forEach(r -> r.cleanup());
		super.cleanup();
	}

	private void startRequestsIfAllowed() {
		for (Iterator<Request> iter = idleRequests.iterator(); iter.hasNext(); ) {
			Request request = iter.next();
			if (request.allowed()) {
				request.act();
				inProgressRequests.add(request);
				iter.remove();
			}
		}
	}

	@Override
	public void act() {
		startRequestsIfAllowed();
	}

	@Override
	public boolean isFinished() {
		startRequestsIfAllowed();
		inProgressRequests.removeIf(r -> r.isFinished());

		return idleRequests.isEmpty() && inProgressRequests.isEmpty();
	}

	@Override
	public String toString() {
		return String.format(
				"ParallelRequest(inProgressRequests = %s, idleRequests = %s)", inProgressRequests, idleRequests);
	}
}
