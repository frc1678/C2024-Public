package com.team1678.frc2024.auto.actions;

import com.team1678.frc2024.Robot;
import com.team1678.frc2024.subsystems.Drive;
import com.team1678.lib.swerve.ChassisSpeeds;
import com.team1678.lib.util.Stopwatch;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.util.TimeDelayedBoolean;

public class TurnInPlaceAction implements Action {

	private final Rotation2d target;
	TimeDelayedBoolean onTarget = new TimeDelayedBoolean();
	private Stopwatch stopwatch;
	private double mTimeToWait;

	public TurnInPlaceAction(Rotation2d _target, double timeToWait) {
		target = _target;
		mTimeToWait = timeToWait;
		stopwatch = new Stopwatch();
	}

	@Override
	public void start() {
		Drive.getInstance().stabilizeHeading(target);
		stopwatch.start();
	}

	@Override
	public void update() {
		Drive.getInstance().stabilizeHeading(Robot.is_red_alliance ? target.mirrorAboutX() : target);
		Drive.getInstance().feedTeleopSetpoint(new ChassisSpeeds());
	}

	@Override
	public boolean isFinished() {
		double heading = Drive.getInstance().getPose().getRotation().getDegrees();
		return onTarget.update(Math.abs(target.getDegrees() - heading) <= 3, 0.05)
				|| stopwatch.getTime() >= mTimeToWait;
	}

	@Override
	public void done() {}
}
