package com.team1678.frc2024.auto.modes.three;

import com.team1678.frc2024.auto.AutoModeBase;
import com.team1678.frc2024.auto.AutoModeEndedException;
import com.team1678.frc2024.auto.actions.LambdaAction;
import com.team1678.frc2024.auto.actions.ParallelAction;
import com.team1678.frc2024.auto.actions.SeriesAction;
import com.team1678.frc2024.auto.actions.SwerveTrajectoryAction;
import com.team1678.frc2024.auto.actions.TurnInPlaceAction;
import com.team1678.frc2024.auto.actions.WaitAction;
import com.team1678.frc2024.auto.actions.WaitForSuperstructureAction;
import com.team1678.frc2024.auto.actions.WaitForTargetTrackAction;
import com.team1678.frc2024.auto.actions.WaitToPassXCoordinateAction;
import com.team1678.frc2024.paths.TrajectoryGenerator;
import com.team1678.frc2024.paths.TrajectoryGenerator.TrajectorySet;
import com.team1678.frc2024.subsystems.Drive;
import com.team1678.frc2024.subsystems.Superstructure;
import com.team254.lib.geometry.Pose2dWithMotion;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.timing.TimedState;
import java.util.List;

public class ThreeNoteMode53 extends AutoModeBase {
	private Drive d = Drive.getInstance();
	private Superstructure s = Superstructure.getInstance();

	Trajectory<TimedState<Pose2dWithMotion>> startToN5Pickup;
	Trajectory<TimedState<Pose2dWithMotion>> N3PickupToNearShot;
	Trajectory<TimedState<Pose2dWithMotion>> N5PickupToFarShot;
	Trajectory<TimedState<Pose2dWithMotion>> shotToN3Pickup;
	Trajectory<TimedState<Pose2dWithMotion>> shotToPreLoadPickup;
	Trajectory<TimedState<Pose2dWithMotion>> preloadPickupToShot;
	Trajectory<TimedState<Pose2dWithMotion>> preloadShotToTeleStart;

	public ThreeNoteMode53() {
		TrajectorySet s = TrajectoryGenerator.getInstance().getTrajectorySet();
		startToN5Pickup = logTrajectory(s.R3StartToN5Pickup);
		N5PickupToFarShot = logTrajectory(s.R3N5PickupToFarShot);
		shotToN3Pickup = logTrajectory(s.R3FarShotToN3Pickup);
		N3PickupToNearShot = logTrajectory(s.R3N3PickupToNearShot);
		shotToPreLoadPickup = logTrajectory(s.R3ShotToPreLoadPickup);
		preloadPickupToShot = logTrajectory(s.R3PreloadPickupToShot);
		preloadShotToTeleStart = logTrajectory(s.R3PreloadShotToTeleStart);
	}

	// spotless:off
	@Override
	protected void routine() throws AutoModeEndedException {
		runAction(new ParallelAction(List.of(
				new SwerveTrajectoryAction(startToN5Pickup, false),
				new SeriesAction(
						new LambdaAction(() -> d.overrideHeading(false)),
						new WaitToPassXCoordinateAction(4.0),
						new LambdaAction(() -> s.intakeToHoldTransition())))));

		s.setWantPrep(true);

		runAction(new ParallelAction(List.of(
				new SwerveTrajectoryAction(N5PickupToFarShot, false),
				new SeriesAction(
						new WaitToPassXCoordinateAction(6.5),
						new LambdaAction(() -> d.overrideHeading(true))))));

		runAction(new LambdaAction(() -> s.tuckState()));
		runAction(new WaitAction(0.15));
		runAction(new WaitForTargetTrackAction(2.0));
		runAction(new LambdaAction(() -> s.fireState()));
		runAction(new WaitForSuperstructureAction(0.35));

		runAction(new ParallelAction(List.of(
				new SwerveTrajectoryAction(shotToN3Pickup, false),
				new SeriesAction(
						new LambdaAction(() -> d.overrideHeading(false)),
						new WaitToPassXCoordinateAction(5.5),
						new LambdaAction(() -> s.intakeToHoldTransition())))));

		runAction(new ParallelAction(List.of(
				new SwerveTrajectoryAction(N3PickupToNearShot, false),
				new SeriesAction(
						new WaitToPassXCoordinateAction(6.5),
						new LambdaAction(() -> d.overrideHeading(true))))));

		runAction(new LambdaAction(() -> s.tuckState()));
		runAction(new WaitAction(0.2));
		runAction(new WaitForTargetTrackAction(2.0));
		runAction(new WaitForSuperstructureAction(0.4));
		d.overrideHeading(false);
		s.intakeToHoldTransition();
		runAction(new TurnInPlaceAction(Rotation2d.fromDegrees(-120.0), 0.5));
		runAction(new SwerveTrajectoryAction(shotToPreLoadPickup, false));
		runAction(new SwerveTrajectoryAction(preloadPickupToShot, false));
		d.overrideHeading(true);
		runAction(new WaitForTargetTrackAction(2.0));
		runAction(new WaitAction(0.2));
		runAction(new LambdaAction(() -> s.fireState()));
		runAction(new WaitForSuperstructureAction(0.4));
		d.overrideHeading(false);
		runAction(new LambdaAction(() -> s.tuckState()));
		runAction(new SwerveTrajectoryAction(preloadShotToTeleStart, false));

		System.out.println("Finished auto!");
	}
	// spotless:on
}
