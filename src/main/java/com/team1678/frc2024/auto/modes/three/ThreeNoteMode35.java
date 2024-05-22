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
import com.team1678.frc2024.subsystems.limelight.Limelight;
import com.team1678.frc2024.subsystems.limelight.Limelight.NotePosition;
import com.team254.lib.geometry.Pose2dWithMotion;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.timing.TimedState;
import java.util.List;

public class ThreeNoteMode35 extends AutoModeBase {
	private Drive d = Drive.getInstance();
	private Superstructure s = Superstructure.getInstance();
	private Limelight n = Limelight.getInstance();

	Trajectory<TimedState<Pose2dWithMotion>> startToN3Pickup;
	Trajectory<TimedState<Pose2dWithMotion>> N5PickupToNearShot;
	Trajectory<TimedState<Pose2dWithMotion>> N3PickupToFarShot;
	Trajectory<TimedState<Pose2dWithMotion>> shotToN5Pickup;
	Trajectory<TimedState<Pose2dWithMotion>> shotToPreLoadPickup;
	Trajectory<TimedState<Pose2dWithMotion>> preloadPickupToShot;
	Trajectory<TimedState<Pose2dWithMotion>> preloadShotToTeleStart;

	Trajectory<TimedState<Pose2dWithMotion>> shotToN4Pickup;
	Trajectory<TimedState<Pose2dWithMotion>> N4PickupToNearShot;

	public ThreeNoteMode35() {
		TrajectorySet s = TrajectoryGenerator.getInstance().getTrajectorySet();
		startToN3Pickup = logTrajectory(s.R3StartToN3Pickup);
		N3PickupToFarShot = logTrajectory(s.R3N3PickupToFarShot);
		shotToN5Pickup = logTrajectory(s.R3FarShotToN5Pickup);
		N5PickupToNearShot = logTrajectory(s.R3N5PickupToNearShot);
		shotToPreLoadPickup = logTrajectory(s.R3ShotToPreLoadPickup);
		preloadPickupToShot = logTrajectory(s.R3PreloadPickupToShot);
		preloadShotToTeleStart = logTrajectory(s.R3PreloadShotToTeleStart);

		shotToN4Pickup = s.R3FarShotToN4Pickup;
		N4PickupToNearShot = s.R3N4PickupToNearShot;
	}

	// spotless:off
	@Override
	protected void routine() throws AutoModeEndedException {
		runAction(new ParallelAction(List.of(
				new SwerveTrajectoryAction(startToN3Pickup, false),
				new SeriesAction(
						new LambdaAction(() -> d.overrideHeading(false)),
						new WaitAction(0.6),
						new LambdaAction(() -> s.intakeToHoldTransition())))));

		s.setWantPrep(true);

		runAction(new ParallelAction(List.of(
			new SwerveTrajectoryAction(N3PickupToFarShot, false),
			new SeriesAction(
					new WaitToPassXCoordinateAction(6.5),
					new LambdaAction(() -> d.overrideHeading(true))))));
					
		runAction(new LambdaAction(() -> s.tuckState()));
		runAction(new WaitAction(0.2));
		runAction(new WaitForTargetTrackAction(2.0));
		runAction(new LambdaAction(() -> s.fireState()));
		runAction(new WaitForSuperstructureAction(0.3));

		boolean hasN5 = n.hasNote(NotePosition.N5);

		runAction(new ParallelAction(List.of(
				new SwerveTrajectoryAction(hasN5 ? shotToN5Pickup : shotToN4Pickup, false),
				new SeriesAction(
						new LambdaAction(() -> d.overrideHeading(false)),
						new WaitToPassXCoordinateAction(5.0),
						new LambdaAction(() -> s.intakeToHoldTransition())))));

		runAction(new ParallelAction(List.of(
				new SwerveTrajectoryAction(hasN5 ? N5PickupToNearShot : N4PickupToNearShot, false),
				new SeriesAction(
						new WaitToPassXCoordinateAction(6.5),
						new LambdaAction(() -> d.overrideHeading(true))))));
						
		runAction(new LambdaAction(() -> s.tuckState()));
		runAction(new WaitAction(0.2));
		runAction(new WaitForTargetTrackAction(2.0));
		runAction(new LambdaAction(() -> s.fireState()));
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
