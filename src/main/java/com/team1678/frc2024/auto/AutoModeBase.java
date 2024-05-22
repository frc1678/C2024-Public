package com.team1678.frc2024.auto;

import com.team1678.frc2024.auto.actions.Action;
import com.team254.lib.geometry.Pose2dWithMotion;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.timing.TimedState;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;

/**
 * An abstract class that is the basis of the robot's autonomous routines. This
 * is implemented in auto modes (which are
 * routines that do actions).
 */
public abstract class AutoModeBase {
	protected double m_update_rate = 1.0 / 50.0;
	protected boolean m_active = false;

	protected double startTime = 0.0;

	protected List<Trajectory<TimedState<Pose2dWithMotion>>> trajectories = new ArrayList<>();

	protected double currentTime() {
		return Timer.getFPGATimestamp() - startTime;
	}

	protected abstract void routine() throws AutoModeEndedException;

	public void run() {
		m_active = true;
		try {
			routine();
		} catch (AutoModeEndedException e) {
			System.out.println("Auto mode done, ended early");
			return;
		}

		done();
		System.out.println("Auto mode done");
	}

	public void done() {}

	public void stop() {
		m_active = false;
	}

	public boolean isActive() {
		return m_active;
	}

	public boolean isActiveWithThrow() throws AutoModeEndedException {
		if (!isActive()) {
			throw new AutoModeEndedException();
		}

		return isActive();
	}

	public void runAction(Action action) throws AutoModeEndedException {
		isActiveWithThrow();
		action.start();

		while (isActiveWithThrow() && !action.isFinished()) {
			action.update();
			long waitTime = (long) (m_update_rate * 1000.0);

			try {
				Thread.sleep(waitTime);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}

		action.done();
	}

	protected Trajectory<TimedState<Pose2dWithMotion>> logTrajectory(
			Trajectory<TimedState<Pose2dWithMotion>> trajectory) {
		trajectories.add(trajectory);
		return trajectory;
	}

	public List<Trajectory<TimedState<Pose2dWithMotion>>> getPaths() {
		return trajectories;
	}
}
