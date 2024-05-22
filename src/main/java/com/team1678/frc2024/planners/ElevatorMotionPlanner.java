package com.team1678.frc2024.planners;

import com.team1678.frc2024.subsystems.Elevator;
import com.team1678.frc2024.subsystems.IntakeDeploy;
import com.team1678.lib.requests.ParallelRequest;
import com.team1678.lib.requests.Prerequisite;
import com.team1678.lib.requests.Request;
import com.team1678.lib.requests.SequentialRequest;

public class ElevatorMotionPlanner {

	public static final double kIntakeClearAngle = IntakeDeploy.kClearAngle;
	public static final double kElevatorClearHeight = 0.273;
	public static final double kElevatorStowTolerance = 0.05;

	private IntakeDeploy mDeploy = IntakeDeploy.getInstance();
	private Elevator mElevator = Elevator.getInstance();

	public Prerequisite intakeClear() {
		return () -> mDeploy.getPosition() < kIntakeClearAngle;
	}

	public Prerequisite elevatorClearUp() {
		return () -> mElevator.getPosition() > kElevatorClearHeight;
	}

	public Prerequisite elevatorClearDown() {
		return () -> mElevator.getPosition() < kElevatorStowTolerance;
	}

	public Request getFullExtendPlan() {
		if (elevatorClearUp().met() || intakeClear().met()) {
			return mElevator.fullExtendRequest();
		} else {
			return new SequentialRequest(
					mDeploy.clearRequest(),
					new ParallelRequest(
							mElevator.fullExtendRequest(), mDeploy.tuckRequest().withPrerequisite(elevatorClearUp())));
		}
	}

	public Request getAmpExtendPlan() {
		if (elevatorClearUp().met() || intakeClear().met()) {
			return mElevator.scoreHeightRequest();
		} else {
			return new SequentialRequest(
					mDeploy.clearRequest(),
					new ParallelRequest(
							mElevator.scoreHeightRequest(),
							mDeploy.tuckRequest().withPrerequisite(elevatorClearUp())));
		}
	}

	public Request getRetractPlan() {
		if (elevatorClearDown().met() || intakeClear().met()) {
			return mElevator.retractRequest();
		} else {
			return new SequentialRequest(
					mDeploy.clearRequest(),
					new SequentialRequest(
							mElevator.retractRequest(), mDeploy.tuckRequest().withPrerequisite(elevatorClearDown())));
		}
	}
}
