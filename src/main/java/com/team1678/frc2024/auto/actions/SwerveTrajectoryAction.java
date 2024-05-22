package com.team1678.frc2024.auto.actions;

import com.team1678.frc2024.subsystems.Drive;
import com.team254.lib.geometry.Pose2dWithMotion;
import com.team254.lib.trajectory.TimedView;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.TimedState;

public class SwerveTrajectoryAction implements Action {

	private Drive mDrive = null;

	private final TrajectoryIterator<TimedState<Pose2dWithMotion>> mTrajectory;
	private final boolean mResetGyro;

	public SwerveTrajectoryAction(Trajectory<TimedState<Pose2dWithMotion>> trajectory) {
		this(trajectory, false);
	}

	public SwerveTrajectoryAction(Trajectory<TimedState<Pose2dWithMotion>> trajectory, boolean resetPose) {
		mTrajectory = new TrajectoryIterator<>(new TimedView<>(trajectory));
		mDrive = Drive.getInstance();
		mResetGyro = resetPose;
	}

	@Override
	public void start() {
		if (mResetGyro) {
			double newRotation = mTrajectory.getState().state().getRotation().getDegrees();
			System.out.println("Reset gyro to " + newRotation);
			mDrive.zeroGyro(newRotation);
		}
		mDrive.setTrajectory(mTrajectory);
	}

	@Override
	public void update() {}

	@Override
	public boolean isFinished() {
		return mDrive.isDoneWithTrajectory();
	}

	@Override
	public void done() {}
}
