/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1678.lib.requests;

/**
 * A state which must be met before a Request can be acted upon
 */
@FunctionalInterface
public interface Prerequisite {
	public abstract boolean met();
}
