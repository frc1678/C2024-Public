package com.team1678.frc2024;

import com.team1678.frc2024.loops.ILooper;
import com.team1678.frc2024.loops.Loop;
import com.team1678.frc2024.loops.Looper;
import com.team1678.frc2024.subsystems.Subsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Used to reset, start, stop, and update all subsystems at once
 */
public class SubsystemManager implements ILooper {
	public static SubsystemManager mInstance = null;

	private List<Subsystem> mAllSubsystems;
	private List<Loop> mLoops = new ArrayList<>();
	private double read_dt = 0.0;
	private double on_loop_dt = 0.0;
	private double write_dt = 0.0;

	private SubsystemManager() {}

	public static SubsystemManager getInstance() {
		if (mInstance == null) {
			mInstance = new SubsystemManager();
		}

		return mInstance;
	}

	public void outputTelemetry() {
		if (Constants.disableExtraTelemetry) {
			return;
		}
		mAllSubsystems.forEach(Subsystem::outputTelemetry);
	}

	public boolean checkSubsystems() {
		boolean ret_val = true;

		for (Subsystem s : mAllSubsystems) {
			ret_val &= s.checkSystem();
		}

		return ret_val;
	}

	public void stop() {
		mAllSubsystems.forEach(Subsystem::stop);
	}

	public List<Subsystem> getSubsystems() {
		return mAllSubsystems;
	}

	public void setSubsystems(Subsystem... allSubsystems) {
		mAllSubsystems = Arrays.asList(allSubsystems);
	}

	private class EnabledLoop implements Loop {
		@Override
		public void onStart(double timestamp) {
			mLoops.forEach(l -> l.onStart(timestamp));
		}

		@Override
		public void onLoop(double timestamp) {
			// Read
			for (int i = 0; i < mAllSubsystems.size(); i++) {
				mAllSubsystems.get(i).readPeriodicInputs();
			}

			// On loop
			for (int i = 0; i < mLoops.size(); i++) {
				mLoops.get(i).onLoop(timestamp);
			}
			on_loop_dt = Timer.getFPGATimestamp() - (timestamp + read_dt);

			// Write
			for (int i = 0; i < mAllSubsystems.size(); i++) {
				mAllSubsystems.get(i).writePeriodicOutputs();
			}
			write_dt = Timer.getFPGATimestamp() - (timestamp + on_loop_dt);

			// Telemetry
			outputTelemetry();
		}

		@Override
		public void onStop(double timestamp) {
			mLoops.forEach(l -> l.onStop(timestamp));
		}
	}

	private class DisabledLoop implements Loop {
		@Override
		public void onStart(double timestamp) {}

		@Override
		public void onLoop(double timestamp) {
			mAllSubsystems.forEach(Subsystem::readPeriodicInputs);
			outputTelemetry();
		}

		@Override
		public void onStop(double timestamp) {}
	}

	public void registerEnabledLoops(Looper enabledLooper) {
		mAllSubsystems.forEach(s -> s.registerEnabledLoops(this));
		enabledLooper.register(new EnabledLoop());
	}

	public void registerDisabledLoops(Looper disabledLooper) {
		disabledLooper.register(new DisabledLoop());
	}

	@Override
	public void register(Loop loop) {
		mLoops.add(loop);
	}

	public void outputLoopTimes() {
		SmartDashboard.putNumber("LooperTimes/ReadDT", read_dt);
		SmartDashboard.putNumber("LooperTimes/OnLoopDT", on_loop_dt);
		SmartDashboard.putNumber("LooperTimes/WriteDT", write_dt);
	}
}
