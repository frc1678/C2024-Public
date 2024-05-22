/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1678.lib.requests;

/**
 * Add your docs here.
 */
public class LambdaRequest extends Request {

	public interface VoidInterface {
		void f();
	}

	VoidInterface mF;

	public LambdaRequest(VoidInterface f) {
		mF = f;
	}

	@Override
	public void act() {
		mF.f();
	}

	@Override
	public String toString() {
		return "LambdaRequest()";
	}
}
