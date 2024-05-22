package com.team1678.frc2024.auto.modes.sfsix;

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
import com.team1678.frc2024.subsystems.vision.VisionDeviceManager;
import com.team1678.lib.util.Stopwatch;
import com.team254.lib.geometry.Pose2dWithMotion;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.timing.TimedState;
import java.util.List;

public class OneThreeSemiFastSixNoteMode extends AutoModeBase {

	Trajectory<TimedState<Pose2dWithMotion>> startToS1Shot;
	Trajectory<TimedState<Pose2dWithMotion>> S1ShotToN1Pickup;
	Trajectory<TimedState<Pose2dWithMotion>> N1PickupToFarShot;
	Trajectory<TimedState<Pose2dWithMotion>> FarShotToN2Pickup;
	Trajectory<TimedState<Pose2dWithMotion>> NearShotToS2Pickup;
	Trajectory<TimedState<Pose2dWithMotion>> S2PickupToS3Pickup;

	private Superstructure s = Superstructure.getInstance();
	private Drive d = Drive.getInstance();

	public OneThreeSemiFastSixNoteMode() {
		TrajectorySet s = TrajectoryGenerator.getInstance().getTrajectorySet();
		startToS1Shot = logTrajectory(s.SF6StartToS1Shot);
		S1ShotToN1Pickup = logTrajectory(s.SF6S1ShotToN1Pickup);
		N1PickupToFarShot = logTrajectory(s.SF6N1PickupToFarShot);
		logTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().SF6FarShotToN3Pickup);
		logTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().SF6N3PickupToNearShot);
		NearShotToS2Pickup = logTrajectory(s.SF6NearShotToS2Pickup);
		S2PickupToS3Pickup = logTrajectory(s.SF6S2PickupToS3Pickup);
	}

	// spotless:off
	@Override
	protected void routine() throws AutoModeEndedException {
		s.setWantPrep(true);
		s.request(IntakeDeploy.getInstance().deployRequest());
		d.setUsePIDControl(true);
		runAction(new ParallelAction(List.of(
				new SwerveTrajectoryAction(startToS1Shot, false),
				new SeriesAction(
						new WaitAction(0.1),
      					new LambdaAction(() -> d.overrideHeading(true)),
						new WaitAction(0.1),
						new LambdaAction(() -> s.slowContinuousShotState())
				))));
		d.overrideHeading(false);
		VisionDeviceManager.setDisableVision(true);
		runAction(new ParallelAction(List.of(
				new SwerveTrajectoryAction(S1ShotToN1Pickup, false),
				new SeriesAction(
					new LambdaAction(() -> s.tuckState()),
                    new WaitAction(0.5),
                    new LambdaAction(() -> d.overrideHeading(false)),
                    new LambdaAction(() -> s.intakeToHoldTransition()),
					new LambdaAction(() -> s.setWantPrep(true))
		))));
		VisionDeviceManager.setDisableVision(false);
		d.setUsePIDControl(false);

		boolean n2_taken = false;
		boolean has_n1 = false;
		Stopwatch n1_timeout = new Stopwatch();
		while (n1_timeout.getTime() < 0.3) {
			n1_timeout.startIfNotRunning();
			if (s.getSerializerBreak()) {
				has_n1 = true;
				break;
			}
		} 
		System.out.println(has_n1 + " " + s.getSerializerBreak());

		if (!has_n1) {
			runAction(new SwerveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().SF6N1ToN2));
			n2_taken = true;	
			runAction(new ParallelAction(List.of(
					new SwerveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().SF6N2PickupToFarShot, false),
					new SeriesAction(
						new WaitAction(0.5),
						new LambdaAction(() -> d.overrideHeading(true))
					)
			)));

		} else {
			runAction(new ParallelAction(List.of(
					new SwerveTrajectoryAction(N1PickupToFarShot, false),
					new SeriesAction(
						new WaitAction(0.5),
						new LambdaAction(() -> d.overrideHeading(true))
					)
			)));
		}

        runAction(new LambdaAction(() -> s.fireState()));
        runAction(new WaitForSuperstructureAction(0.4));
		d.overrideHeading(false);
		
		VisionDeviceManager.setDisableVision(true);
		runAction(new ParallelAction(List.of(
			new SwerveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().SF6FarShotToN3Pickup, false),
			new SeriesAction(
					new WaitAction(0.3),
					new LambdaAction(() -> s.intakeToHoldTransition())))
		));
		d.overrideHeading(true);
		VisionDeviceManager.setDisableVision(false);
		runAction(new SwerveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().SF6N3PickupToNearShot, false));

		s.setWantPrep(true);
		runAction(new WaitAction(0.1));
		runAction(new WaitForTargetTrackAction(2.0));
		s.fireState();
		runAction(new WaitForSuperstructureAction(1.0));
		s.continuousShootState();

		VisionDeviceManager.setDisableVision(true);
		runAction(new SwerveTrajectoryAction(NearShotToS2Pickup, false));
		VisionDeviceManager.setDisableVision(false);
		runAction(new WaitAction(0.5));
		runAction(new SwerveTrajectoryAction(S2PickupToS3Pickup, false));

		System.out.println("Finished auto!");
	}
	// spotless:on
}
