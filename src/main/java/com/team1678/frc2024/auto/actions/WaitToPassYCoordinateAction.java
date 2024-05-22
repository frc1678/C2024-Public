package com.team1678.frc2024.auto.actions;

import com.team1678.frc2024.subsystems.Drive;

public class WaitToPassYCoordinateAction implements Action {
	double startingXCoordinate;
	double targetXCoordinate;
	Drive drive;

	public WaitToPassYCoordinateAction(double y) {
		targetXCoordinate = y;
		drive = Drive.getInstance();
	}

	@Override
	public boolean isFinished() {
		return Math.signum(startingXCoordinate - targetXCoordinate)
				!= Math.signum(drive.getPose().getTranslation().y() - targetXCoordinate);
	}

	@Override
	public void start() {
		startingXCoordinate = drive.getPose().getTranslation().y();
	}

	@Override
	public void update() {}

	@Override
	public void done() {}
}
