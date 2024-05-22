package com.team1678.frc2024.auto.modes.legacy;

import com.team1678.frc2024.auto.AutoModeBase;
import com.team1678.frc2024.auto.AutoModeEndedException;
import com.team1678.frc2024.auto.actions.LambdaAction;
import com.team1678.frc2024.auto.actions.ParallelAction;
import com.team1678.frc2024.auto.actions.SeriesAction;
import com.team1678.frc2024.auto.actions.SwerveTrajectoryAction;
import com.team1678.frc2024.auto.actions.WaitAction;
import com.team1678.frc2024.auto.actions.WaitForSuperstructureAction;
import com.team1678.frc2024.auto.actions.WaitForTargetTrackAction;
import com.team1678.frc2024.paths.TrajectoryGenerator;
import com.team1678.frc2024.paths.TrajectoryGenerator.TrajectorySet;
import com.team1678.frc2024.subsystems.Drive;
import com.team1678.frc2024.subsystems.IntakeDeploy;
import com.team1678.frc2024.subsystems.Superstructure;
import com.team254.lib.geometry.Pose2dWithMotion;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.timing.TimedState;
import java.util.List;

public class AlternateSixNote extends AutoModeBase {

	Trajectory<TimedState<Pose2dWithMotion>> rightStartToRightPickup;
	Trajectory<TimedState<Pose2dWithMotion>> rightPickupToMidShot;
	Trajectory<TimedState<Pose2dWithMotion>> midShotToMidPickup;
	Trajectory<TimedState<Pose2dWithMotion>> midShotToLeftShot;
	Trajectory<TimedState<Pose2dWithMotion>> leftShotToFirstCenterPickup;
	Trajectory<TimedState<Pose2dWithMotion>> firstCenterPickupToLeftShot;
	Trajectory<TimedState<Pose2dWithMotion>> leftShotToSecondCenterPickup;
	Trajectory<TimedState<Pose2dWithMotion>> secondCenterPickupToLeftShot;

	private Superstructure s = Superstructure.getInstance();
	private Drive d = Drive.getInstance();

	public AlternateSixNote() {
		TrajectorySet s = TrajectoryGenerator.getInstance().getTrajectorySet();
		rightStartToRightPickup = logTrajectory(s.L6RightStartToRightShot);
		rightPickupToMidShot = logTrajectory(s.L6RightPickupToMidShot);
		midShotToLeftShot = logTrajectory(s.L6MidPickupToLeftShot);
		leftShotToFirstCenterPickup = logTrajectory(s.AL6LeftShotToCenterPickup);
		firstCenterPickupToLeftShot = logTrajectory(s.AL6CenterPickupToLeftShot);
		leftShotToSecondCenterPickup = logTrajectory(s.AL6LeftShotToSecondCenterPickup);
		secondCenterPickupToLeftShot = logTrajectory(s.AL6SecondCenterPickupToLeftShot);
	}

	// spotless:off
	@Override
	protected void routine() throws AutoModeEndedException {
		s.setWantPrep(true);
		d.overrideHeading(true);
		s.request(IntakeDeploy.getInstance().deployRequest());
		runAction(new ParallelAction(List.of(
				new SwerveTrajectoryAction(rightStartToRightPickup, false),
				new SeriesAction(
						new WaitAction(0.05),
						new WaitForTargetTrackAction(0.5),
						new LambdaAction(() -> s.continuousShootState())))));

		runAction(new SwerveTrajectoryAction(rightPickupToMidShot, false));
		runAction(new SwerveTrajectoryAction(midShotToLeftShot, false));

		runAction(new ParallelAction(List.of(
				new SwerveTrajectoryAction(leftShotToFirstCenterPickup, false),
				new SeriesAction(
						new WaitAction(0.1),
						new LambdaAction(() -> s.idleState()),
						new LambdaAction(() -> s.intakeToHoldTransition()),
						new WaitAction(0.3),
						new LambdaAction(() -> s.setWantPrep(false)),
						new LambdaAction(() -> d.overrideHeading(false))))));

		d.overrideHeading(true);
		s.setWantPrep(true);
		runAction(new SwerveTrajectoryAction(firstCenterPickupToLeftShot, false));
		runAction(new WaitForTargetTrackAction(2.0));
		runAction(new WaitAction(0.1));
		s.fireState();
		runAction(new WaitForSuperstructureAction(1.0));
		d.overrideHeading(false);

		runAction(new ParallelAction(List.of(
				new SwerveTrajectoryAction(leftShotToSecondCenterPickup, false),
				new SeriesAction(new WaitAction(0.5), new LambdaAction(() -> s.intakeToHoldTransition())))));

		d.overrideHeading(true);
		runAction(new SwerveTrajectoryAction(secondCenterPickupToLeftShot, false));
		runAction(new WaitForTargetTrackAction(2.0));
		runAction(new WaitAction(0.2));
		s.fireState();

		System.out.println("Finished auto!");
	}
	// spotless:on
}
