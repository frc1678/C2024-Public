package com.team1678.frc2024.auto.modes;

import com.team1678.frc2024.auto.AutoModeBase;
import com.team1678.frc2024.auto.AutoModeEndedException;
import com.team1678.frc2024.auto.actions.SwerveTrajectoryAction;
import com.team1678.frc2024.paths.TrajectoryGenerator;
import com.team254.lib.geometry.Pose2dWithMotion;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.timing.TimedState;

public class TestPathMode extends AutoModeBase {

	Trajectory<TimedState<Pose2dWithMotion>> trajectory1 =
			logTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().testTrajectory);
	Trajectory<TimedState<Pose2dWithMotion>> trajectory2 =
			logTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().testTrajectory2);

	public TestPathMode() {}

	// spotless:off
	@Override
	protected void routine() throws AutoModeEndedException {
		System.out.println("Running test mode auto!");
		runAction(new SwerveTrajectoryAction(trajectory1, true));
		runAction(new SwerveTrajectoryAction(trajectory2, false));
		System.out.println("Finished auto!");
	}
	// spotless:on
}
