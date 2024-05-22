package com.team1678.frc2024.auto.modes.fiveline;

import com.team1678.frc2024.auto.AutoModeBase;
import com.team1678.frc2024.auto.AutoModeEndedException;
import com.team1678.frc2024.auto.actions.LambdaAction;
import com.team1678.frc2024.auto.actions.ParallelAction;
import com.team1678.frc2024.auto.actions.SeriesAction;
import com.team1678.frc2024.auto.actions.SwerveTrajectoryAction;
import com.team1678.frc2024.auto.actions.WaitAction;
import com.team1678.frc2024.auto.actions.WaitForSuperstructureAction;
import com.team1678.frc2024.auto.actions.WaitToPassXCoordinateAction;
import com.team1678.frc2024.paths.TrajectoryGenerator;
import com.team1678.frc2024.paths.TrajectoryGenerator.TrajectorySet;
import com.team1678.frc2024.subsystems.Drive;
import com.team1678.frc2024.subsystems.IntakeDeploy;
import com.team1678.frc2024.subsystems.Superstructure;
import com.team254.lib.geometry.Pose2dWithMotion;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.timing.TimedState;
import java.util.List;

public class N5FiveLineMode extends AutoModeBase {

	Trajectory<TimedState<Pose2dWithMotion>> startToN5Pickup;
	Trajectory<TimedState<Pose2dWithMotion>> N5PickupToNearShot;
	Trajectory<TimedState<Pose2dWithMotion>> nearShotToPreloadPickup;
	Trajectory<TimedState<Pose2dWithMotion>> preloadPickupToShot;
	Trajectory<TimedState<Pose2dWithMotion>> preloadShotToS3Pickup;
	Trajectory<TimedState<Pose2dWithMotion>> S3PickupToS2Pickup;
	Trajectory<TimedState<Pose2dWithMotion>> S2PickupToS1Pickup;
	Trajectory<TimedState<Pose2dWithMotion>> S1PickupToTeleStart;

	private Superstructure s = Superstructure.getInstance();
	private Drive d = Drive.getInstance();

	public N5FiveLineMode() {
		TrajectorySet s = TrajectoryGenerator.getInstance().getTrajectorySet();
		startToN5Pickup = logTrajectory(s.R5StartToN5Pickup);
		N5PickupToNearShot = logTrajectory(s.R5N5PickupToNearShot);
		nearShotToPreloadPickup = logTrajectory(s.R5NearShotToPreloadPickup);
		preloadPickupToShot = logTrajectory(s.R5PreloadPickupToShot);
		preloadShotToS3Pickup = logTrajectory(s.R5PreloadShotToS3Pickup);
		S3PickupToS2Pickup = logTrajectory(s.R5S3PickupToS2Pickup);
		S2PickupToS1Pickup = logTrajectory(s.R5S2PickupToS1Pickup);
		S1PickupToTeleStart = logTrajectory(s.R5SS1PickupToTeleStart);
	}

	// spotless:off
	@Override
	protected void routine() throws AutoModeEndedException {
		runAction(new ParallelAction(List.of(
				new SwerveTrajectoryAction(startToN5Pickup, false),
				new SeriesAction(
						new LambdaAction(() -> d.overrideHeading(false)),
						new WaitToPassXCoordinateAction(5.0),
						new LambdaAction(() -> s.intakeToHoldTransition())))));

		s.setWantPrep(true);

		runAction(new ParallelAction(List.of(
				new SwerveTrajectoryAction(N5PickupToNearShot, false),
				new SeriesAction(new WaitAction(0.5), new LambdaAction(() -> d.overrideHeading(true))))));

		s.fireState();
		runAction(new WaitForSuperstructureAction(0.4));

		d.overrideHeading(false);
		s.intakeToHoldTransition();
		runAction(new SwerveTrajectoryAction(nearShotToPreloadPickup, false));

		d.overrideHeading(true);
		runAction(new SwerveTrajectoryAction(preloadPickupToShot));
		s.request(IntakeDeploy.getInstance().deployRequest());
		runAction(new WaitAction(0.1));
		s.fireState();
		runAction(new WaitForSuperstructureAction(0.3));

		s.slowContinuousShotState();
		runAction(new SwerveTrajectoryAction(preloadShotToS3Pickup, false));
		runAction(new WaitAction(0.2));
		runAction(new SwerveTrajectoryAction(S3PickupToS2Pickup, false));
		runAction(new WaitAction(0.2));
		runAction(new SwerveTrajectoryAction(S2PickupToS1Pickup, false));
		runAction(new WaitAction(0.4));

		s.tuckState();

		runAction(new ParallelAction(List.of(
				new SwerveTrajectoryAction(S1PickupToTeleStart, false),
				new SeriesAction(
						new WaitAction(0.2),
						new LambdaAction(() -> d.overrideHeading(false)),
						new LambdaAction(() -> s.setWantPrep(false)),
						new LambdaAction(() -> s.idleState())))));
		System.out.println("Finished auto!");
	}
	// spotless:on
}
