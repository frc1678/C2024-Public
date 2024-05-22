package com.team1678.frc2024.auto.actions;

import com.team1678.frc2024.subsystems.Drive;
import com.team1678.lib.swerve.ChassisSpeeds;
import com.team254.lib.util.TimeDelayedBoolean;

public class WaitForHeadingAction implements Action {
	Drive drive;
	double target;
	double margin;
	TimeDelayedBoolean onTarget = new TimeDelayedBoolean();

	public WaitForHeadingAction(double target, double margin) {
		drive = Drive.getInstance();
		this.target = target;
		this.margin = margin;
	}

	@Override
	public boolean isFinished() {
		double heading = drive.getPose().getRotation().getDegrees();
		System.out.println(heading);
		return onTarget.update(Math.abs(target - heading) <= margin, 1.0);
	}

	@Override
	public void start() {}

	@Override
	public void update() {
		drive.feedTeleopSetpoint(new ChassisSpeeds());
	}

	@Override
	public void done() {}
}
