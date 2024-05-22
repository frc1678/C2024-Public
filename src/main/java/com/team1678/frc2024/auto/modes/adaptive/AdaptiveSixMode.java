package com.team1678.frc2024.auto.modes.adaptive;

import com.team1678.frc2024.auto.AutoModeBase;
import com.team1678.frc2024.auto.AutoModeEndedException;
import com.team1678.frc2024.auto.AutoModeSelector.TargetNote;
import com.team1678.frc2024.auto.AutoModeSelector.TargetSpike;
import com.team1678.frc2024.auto.actions.LambdaAction;
import com.team1678.frc2024.auto.actions.ParallelAction;
import com.team1678.frc2024.auto.actions.SeriesAction;
import com.team1678.frc2024.auto.actions.SwerveTrajectoryAction;
import com.team1678.frc2024.auto.actions.WaitAction;
import com.team1678.frc2024.auto.actions.WaitForSuperstructureAction;
import com.team1678.frc2024.paths.TrajectoryGenerator;
import com.team1678.frc2024.paths.TrajectoryGenerator.TrajectorySet;
import com.team1678.frc2024.subsystems.Drive;
import com.team1678.frc2024.subsystems.IntakeDeploy;
import com.team1678.frc2024.subsystems.Superstructure;
import com.team1678.frc2024.subsystems.vision.VisionDeviceManager;
import com.team1678.lib.util.Stopwatch;
import java.util.List;

public class AdaptiveSixMode extends AutoModeBase {

	private final TargetNote kFirstNote, kSecondNote;
	private final TargetSpike kSpikeMode;
	private final TrajectorySet t;

	private final Drive d = Drive.getInstance();
	private final Superstructure s = Superstructure.getInstance();

	private boolean mN1Taken = false;
	private boolean mN2Taken = false;
	private boolean mN3Taken = false;

	public AdaptiveSixMode(TargetNote first_note, TargetNote second_note, TargetSpike spike_mode) {
		t = TrajectoryGenerator.getInstance().getTrajectorySet();
		kFirstNote = first_note;
		kSecondNote = second_note;
		kSpikeMode = spike_mode;
	}

	// spotless:off
    private void runS2ToN1ToFarShot() throws AutoModeEndedException {
        runAction(new ParallelAction(List.of(
            new SwerveTrajectoryAction(t.A6S2ToN1),
            new SeriesAction(
                	// new LambdaAction(() -> s.tuckState()),
                    new WaitAction(0.5),
                    new LambdaAction(() -> d.overrideHeading(false)),
                    new LambdaAction(() -> s.intakeToHoldTransition())
            )
        )));
        d.setUsePIDControl(false);
        mN1Taken = true;
		Stopwatch note_timeout = new Stopwatch();
        boolean has_n1 = false;
		while (note_timeout.getTime() < 0.3) {
			note_timeout.startIfNotRunning();
			if (s.getSerializerBreak()) {
				has_n1 = true;
				break;
			}
		} 

        if (!has_n1) {
            runAction(new SwerveTrajectoryAction(t.A6N1ToN2));
            mN2Taken = true;
            runAction(new ParallelAction(List.of(
                new SwerveTrajectoryAction(t.A6N2ToFarShot),
                new SeriesAction(
                    new WaitAction(0.5),
                    new LambdaAction(() -> d.overrideHeading(true))
                )
            )));
        } else {
            runAction(new ParallelAction(List.of(
                new SwerveTrajectoryAction(t.A6N1ToFarShot, false),
                new SeriesAction(
                    new WaitAction(0.5),
                    new LambdaAction(() -> d.overrideHeading(true))
                )
            )));
        }
        runAction(new WaitAction(0.1));
        runAction(new LambdaAction(() -> s.fireState()));
        runAction(new WaitForSuperstructureAction(0.4));
		d.overrideHeading(false);
    }

    private void runS2ToN2ToFarShot() throws AutoModeEndedException {
        runAction(new ParallelAction(List.of(
            new SwerveTrajectoryAction(t.A6S2ToN2),
            new SeriesAction(
                	new LambdaAction(() -> s.tuckState()),
                    new WaitAction(0.5),
                    new LambdaAction(() -> d.overrideHeading(false)),
                    new LambdaAction(() -> s.intakeToHoldTransition())
            )
        )));
        d.setUsePIDControl(false);
        mN2Taken = true;
		Stopwatch note_timeout = new Stopwatch();
        boolean has_n2 = false;
		while (note_timeout.getTime() < 0.3) {
			note_timeout.startIfNotRunning();
			if (s.getSerializerBreak()) {
				has_n2 = true;
				break;
			}
		} 

        if (!has_n2) {
            if (kSecondNote.equals(TargetNote.N1)) {
                runAction(new SwerveTrajectoryAction(t.A6N2ToN1));
                mN1Taken = true;
                runAction(new ParallelAction(List.of(
                    new SwerveTrajectoryAction(t.A6N1ToFarShot),
                    new SeriesAction(
                        new WaitAction(0.5),
                        new LambdaAction(() -> d.overrideHeading(true))
                    )
                )));
                runAction(new WaitAction(0.1));
            } else {
                runAction(new SwerveTrajectoryAction(t.A6N2ToN3));
                mN3Taken = true;
                runAction(new ParallelAction(List.of(
                    new SwerveTrajectoryAction(t.A6N3ToFarShot),
                    new SeriesAction(
                        new WaitAction(0.5),
                        new LambdaAction(() -> d.overrideHeading(true))
                    )
                )));
                runAction(new WaitAction(0.1));
            }

        } else {
            runAction(new ParallelAction(List.of(
                new SwerveTrajectoryAction(t.A6N2ToFarShot, false),
                new SeriesAction(
                    new WaitAction(0.5),
                    new LambdaAction(() -> d.overrideHeading(true))
                )
            )));
        }
        runAction(new WaitAction(0.1));
        runAction(new LambdaAction(() -> s.fireState()));
        runAction(new WaitForSuperstructureAction(0.4));
		d.overrideHeading(false);
    }

    private void runS2ToN3ToFarShot() throws AutoModeEndedException {
       runAction(new ParallelAction(List.of(
            new SwerveTrajectoryAction(t.A6S2ToN3),
            new SeriesAction(
                	new LambdaAction(() -> s.tuckState()),
                    new WaitAction(0.5),
                    new LambdaAction(() -> d.overrideHeading(false)),
                    new LambdaAction(() -> s.intakeToHoldTransition())
            )
        )));
        d.setUsePIDControl(false);
        mN3Taken = true;
		Stopwatch note_timeout = new Stopwatch();
        boolean has_n3 = false;
		while (note_timeout.getTime() < 0.3) {
			note_timeout.startIfNotRunning();
			if (s.getSerializerBreak()) {
				has_n3 = true;
				break;
			}
		} 

        if (!has_n3) {
            runAction(new SwerveTrajectoryAction(t.A6N3ToN2));
            mN2Taken = true;
            runAction(new ParallelAction(List.of(
                new SwerveTrajectoryAction(t.A6N2ToFarShot),
                new SeriesAction(
                    new WaitAction(0.5),
                    new LambdaAction(() -> d.overrideHeading(true))
                )
            )));
            runAction(new WaitAction(0.1));
        } else {
            runAction(new ParallelAction(List.of(
                new SwerveTrajectoryAction(t.A6N3ToFarShot, false),
                new SeriesAction(
                    new WaitAction(0.5),
                    new LambdaAction(() -> d.overrideHeading(true))
                )
            )));
        }
        runAction(new WaitAction(0.1));
        runAction(new LambdaAction(() -> s.fireState()));
        runAction(new WaitForSuperstructureAction(0.4));
		d.overrideHeading(false);
    }

    private void runFarShotToN1ToFarShot() throws AutoModeEndedException {
        runAction(new ParallelAction(List.of(
            new SwerveTrajectoryAction(t.A6FarShotToN1, false),
            new SeriesAction(
                new WaitAction(0.2),
                new LambdaAction(() -> d.overrideHeading(false)),
                new WaitAction(0.1),
                new LambdaAction(() -> s.intakeToHoldTransition())))));

        d.overrideHeading(true);
        runAction(new SwerveTrajectoryAction(t.A6N1ToFarShot, false));
        runAction(new LambdaAction(() -> s.fireState()));
        runAction(new WaitForSuperstructureAction(0.4));
		d.overrideHeading(false);
    }

    private void runFarShotToN2ToFarShot() throws AutoModeEndedException {
        runAction(new ParallelAction(List.of(
            new SwerveTrajectoryAction(t.A6FarShotToN2, false),
            new SeriesAction(
                new WaitAction(0.2),
                new LambdaAction(() -> d.overrideHeading(false)),
                new WaitAction(0.1),
                new LambdaAction(() -> s.intakeToHoldTransition())))));

        d.overrideHeading(true);
        runAction(new SwerveTrajectoryAction(t.A6N2ToFarShot, false));
        runAction(new LambdaAction(() -> s.fireState()));
        runAction(new WaitForSuperstructureAction(0.4));
		d.overrideHeading(false);
    }

    private void runFarShotToN3ToFarShot() throws AutoModeEndedException {
        runAction(new ParallelAction(List.of(
            new SwerveTrajectoryAction(t.A6FarShotToN3, false),
            new SeriesAction(
                new WaitAction(0.2),
                new LambdaAction(() -> d.overrideHeading(false)),
                new WaitAction(0.1),
                new LambdaAction(() -> s.intakeToHoldTransition())))));

        d.overrideHeading(true);
        runAction(new SwerveTrajectoryAction(t.A6N3ToFarShot, false));
        runAction(new LambdaAction(() -> s.fireState()));
        runAction(new WaitForSuperstructureAction(0.4));
		d.overrideHeading(false);
    }

    private void runFarShotToN1ToNearShot() throws AutoModeEndedException {
        runAction(new ParallelAction(List.of(
            new SwerveTrajectoryAction(t.A6FarShotToN1, false),
            new SeriesAction(
                new WaitAction(0.2),
                new LambdaAction(() -> d.overrideHeading(false)),
                new WaitAction(0.1),
                new LambdaAction(() -> s.intakeToHoldTransition())))));

        d.overrideHeading(true);
        runAction(new SwerveTrajectoryAction(t.A6N1ToNearShot, false));
        runAction(new LambdaAction(() -> s.continuousShootState()));
        runAction(new WaitAction(0.2));
    }

    private void runFarShotToN2ToNearShot() throws AutoModeEndedException {
         runAction(new ParallelAction(List.of(
            new SwerveTrajectoryAction(t.A6FarShotToN2, false),
            new SeriesAction(
                new WaitAction(0.2),
                new LambdaAction(() -> d.overrideHeading(false)),
                new WaitAction(0.1),
                new LambdaAction(() -> s.intakeToHoldTransition())))));

        d.overrideHeading(true);
        runAction(new SwerveTrajectoryAction(t.A6N2ToNearShot, false));
        runAction(new LambdaAction(() -> s.continuousShootState()));
        runAction(new WaitAction(0.2));
    }

    private void runFarShotToN3ToNearShot() throws AutoModeEndedException {
         runAction(new ParallelAction(List.of(
            new SwerveTrajectoryAction(t.A6FarShotToN3, false),
            new SeriesAction(
                new WaitAction(0.2),
                new LambdaAction(() -> d.overrideHeading(false)),
                new WaitAction(0.1),
                new LambdaAction(() -> s.intakeToHoldTransition())))));

        d.overrideHeading(true);
        runAction(new SwerveTrajectoryAction(t.A6N3ToNearShot, false));
        runAction(new LambdaAction(() -> s.continuousShootState()));
        runAction(new WaitAction(0.2));
    }

    private void runNearShotToLeftSpikeSequence() throws AutoModeEndedException {
        s.slowContinuousShotState();
        runAction(new WaitAction(0.2));
        VisionDeviceManager.setDisableVision(true);
        runAction(new SwerveTrajectoryAction(t.A6NearShotToS1));
        runAction(new SwerveTrajectoryAction(t.A6S1ToS3));
        runAction(new WaitAction(0.5));
        d.setUsePIDControl(true);
        d.overrideHeading(false);
        s.tuckState();
        runAction(new SwerveTrajectoryAction(t.A6S3ToDriveout));
    }

    private void runNearShotToRightSpikeSequence() throws AutoModeEndedException {
        s.slowContinuousShotState();
        runAction(new WaitAction(0.2));
        VisionDeviceManager.setDisableVision(true);
        runAction(new SwerveTrajectoryAction(t.A6NearShotToS3));
        runAction(new SwerveTrajectoryAction(t.A6S3ToS1));
        runAction(new WaitAction(0.5));
        d.setUsePIDControl(true);
        d.overrideHeading(false);
        s.tuckState();
        runAction(new SwerveTrajectoryAction(t.A6S1ToDriveout));
    }

    
    @Override
    protected void routine() throws AutoModeEndedException {
        s.setFerry(false);
        s.setWantPrep(true);
        d.overrideHeading(true);
        d.setUsePIDControl(true);
		s.request(IntakeDeploy.getInstance().deployRequest());
        runAction(new ParallelAction(List.of(
            new SwerveTrajectoryAction(t.A6StartToS2),
            new SeriesAction(
                new WaitAction(0.05),
                new LambdaAction(() -> s.slowContinuousShotState())
            )
        )));
        switch (kFirstNote) {
            case N3:
                runS2ToN3ToFarShot();
                break;
            case N2:
                runS2ToN2ToFarShot();
                break;
            case N1: 
            default:
                runS2ToN1ToFarShot();
                break;
        }
        switch (kSecondNote) {
            case N3:
                if (mN3Taken) {
                    if (mN2Taken) {
                        runFarShotToN1ToNearShot();
                    } else {
                        runFarShotToN2ToNearShot();
                    }
                } else {
                    runFarShotToN3ToNearShot();
                }
                break;
            case N2:
                if (mN2Taken) {
                    if (mN1Taken) {
                        runFarShotToN3ToNearShot();
                    } else {
                        runFarShotToN1ToNearShot();
                    }
                } else {
                    runFarShotToN2ToNearShot();
                }
                break;
            case N1: 
            default:
                if (mN1Taken) {
                    if (mN2Taken) {
                        runFarShotToN3ToNearShot();
                    } else {
                        runFarShotToN2ToNearShot();
                    }
                } else {
                    runFarShotToN1ToNearShot();
                }
                break;
        }
        switch (kSpikeMode) {
            case RIGHT:
                runNearShotToRightSpikeSequence();
                break;
            case LEFT: 
            default:
                runNearShotToLeftSpikeSequence();
                break;
        }
    }
    // spotless:on
}
