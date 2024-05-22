package com.team1678.frc2024.auto.modes.legacy;

import com.team1678.frc2024.auto.AutoModeBase;
import com.team1678.frc2024.auto.AutoModeEndedException;
import com.team1678.frc2024.auto.actions.TurnInPlaceAction;
import com.team1678.frc2024.auto.actions.WaitAction;
import com.team1678.frc2024.auto.actions.WaitForTargetTrackAction;
import com.team1678.frc2024.subsystems.Drive;
import com.team1678.frc2024.subsystems.Superstructure;
import com.team1678.lib.swerve.ChassisSpeeds;
import com.team254.lib.geometry.Rotation2d;

public class ShootAndDriveMode extends AutoModeBase {

	private Superstructure s = Superstructure.getInstance();
	private Drive d = Drive.getInstance();

	public ShootAndDriveMode() {}

	// spotless:off
	@Override
	protected void routine() throws AutoModeEndedException {
		System.out.println("Shoot + Drive Mode");
		d.overrideHeading(true);
		s.setWantPrep(true);
		runAction(new WaitForTargetTrackAction(1.0));
		s.fireState();
		runAction(new WaitAction(2.0));
		s.setWantPrep(false);
		d.overrideHeading(false);
		runAction(new TurnInPlaceAction(Rotation2d.fromDegrees(0.0), 2.0));
		runAction(new WaitAction(6.0));
		d.feedTeleopSetpoint(new ChassisSpeeds(2.0, 0.0, 0.0));
		runAction(new WaitAction(1.0));
		d.feedTeleopSetpoint(new ChassisSpeeds(0.0, 0.0, 0.0));
		Drive.getInstance().stop();
	}
	// spotless:on
}
