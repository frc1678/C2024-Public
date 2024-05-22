package com.team1678.frc2024.loops;

import com.team254.lib.geometry.Pose2dWithMotion;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.TimedView;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.TimedState;
import java.util.ArrayList;
import java.util.List;

public class QuinticPathTransmitter implements Loop {
	private static QuinticPathTransmitter instance = new QuinticPathTransmitter();

	public static QuinticPathTransmitter getInstance() {
		return instance;
	}

	public QuinticPathTransmitter() {}

	private List<TrajectoryIterator<TimedState<Pose2dWithMotion>>> remainingTrajectories = new ArrayList<>();
	private TrajectoryIterator<TimedState<Pose2dWithMotion>> currentTrajectory;
	private double t = 0;
	private boolean defaultCookReported = false;

	private double minAccel = 0.0;

	private double startingTime = 0.0;

	public void addPath(Trajectory<TimedState<Pose2dWithMotion>> path) {
		remainingTrajectories.add(new TrajectoryIterator<>(new TimedView<>(path)));
		currentTrajectory = null;
	}

	public void addPaths(List<Trajectory<TimedState<Pose2dWithMotion>>> paths) {
		paths.forEach((p) -> addPath(p));
	}

	public void clearPaths() {
		remainingTrajectories = new ArrayList<>();
		currentTrajectory = null;
	}

	public double getTotalPathTime(List<Trajectory<TimedState<Pose2dWithMotion>>> paths) {
		double total = 0.0;

		for (Trajectory<TimedState<Pose2dWithMotion>> path : paths) {
			total += path.getLastPoint().state().t();
		}

		return total;
	}

	@Override
	public void onStart(double timestamp) {}

	@Override
	public void onLoop(double timestamp) {
		if (currentTrajectory == null) {
			if (remainingTrajectories.isEmpty()) {
				return;
			}

			currentTrajectory = remainingTrajectories.remove(0);
			defaultCookReported = false;
			t = 0;
			startingTime = timestamp;
		}

		t = timestamp - startingTime;
		TimedState<Pose2dWithMotion> state = currentTrajectory.preview(t).state();
		Translation2d pos = state.state().getTranslation();
		Rotation2d rot = state.state().getRotation();

		if (state.acceleration() < minAccel) minAccel = state.acceleration();

		if (state.acceleration() < 0.0 && !defaultCookReported) {
			defaultCookReported = true;
		}

		if (t >= currentTrajectory.trajectory().getLastPoint().state().t()) {
			currentTrajectory = null;
		}
	}

	@Override
	public void onStop(double timestamp) {}
}
