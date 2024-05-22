package com.team1678.frc2024.subsystems.limelight;

import com.team1678.frc2024.Constants;
import com.team1678.frc2024.Constants.LimelightConstants;
import com.team1678.frc2024.FieldLayout;
import com.team1678.frc2024.subsystems.Drive;
import com.team1678.frc2024.subsystems.LEDs;
import com.team1678.frc2024.subsystems.Subsystem;
import com.team1678.frc2024.subsystems.limelight.LimelightHelpers.LimelightResults;
import com.team1678.lib.logger.LogUtil;
import com.team1678.lib.util.Stopwatch;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Limelight extends Subsystem {
	private final String name;
	private final Pose2d robotToCameraTransform;
	private final double cameraHeightMeters;
	private final Rotation2d cameraPitch, cameraYaw;

	private boolean isConnected = false;
	private static Limelight mInstance = null;

	// Init using our constants
	public static Limelight getInstance() {
		if (mInstance == null) {
			mInstance = new Limelight(
					LimelightConstants.kName,
					LimelightConstants.kRobotToCameraTranslation,
					LimelightConstants.kCameraHeightMeters,
					LimelightConstants.kCameraPitch,
					LimelightConstants.kCameraYaw);
		}
		return mInstance;
	}

	// Default Init
	public Limelight(
			String name,
			Translation2d robotToCameraTranslation,
			double cameraHeightMeters,
			Rotation2d cameraPitch,
			Rotation2d cameraYaw) {
		this.name = name;
		this.robotToCameraTransform = new Pose2d(robotToCameraTranslation, Rotation2d.identity());
		this.cameraHeightMeters = cameraHeightMeters;
		this.cameraPitch = cameraPitch;
		this.cameraYaw = cameraYaw;
	}

	private GoalTracker noteTracker = new GoalTracker(Constants.LimelightConstants.kNoteTrackerConstants);
	private List<Translation2d> robotToNotes = new ArrayList<Translation2d>();
	private LEDs mLEDs = LEDs.getInstance();

	private final LimelightInputs inputs = new LimelightInputs();
	private final Stopwatch lastUpdateStopwatch = new Stopwatch();
	private double previousHeartbeat = -1.0;

	/* Periodic of the Limelight */

	@Override
	public void readPeriodicInputs() {
		double timestamp = Timer.getFPGATimestamp();
		if (RobotBase.isReal()) {
			inputs.heartbeat = LimelightHelpers.getLimelightNTDouble(name, "hb");
			inputs.jsonDump = LimelightHelpers.getJSONDump(name);
		}

		inputs.results = LimelightHelpers.getLatestResults(name, inputs.jsonDump);

		if (inputs.heartbeat != previousHeartbeat) {
			isConnected = true;
			previousHeartbeat = inputs.heartbeat;
			double observationTime = timestamp - getTotalLatencySeconds(inputs.results);

			Translation2d robot_translation = Drive.getInstance().getPose().getTranslation();

			List<Translation2d> targetPositions = Arrays.stream(inputs.results.targetingResults.targets_Detector)
					.map(target -> {
						TargetInfo targetInfo = new TargetInfo(
								Rotation2d.fromDegrees(-target.tx).tan(),
								Rotation2d.fromDegrees(target.ty).tan());
						Translation2d targetPosition =
								getCameraToGoalTranslation(targetInfo, Constants.LimelightConstants.kNoteHeight);

						if (targetPosition == null) {
							return null;
						}

						robotToNotes.add(targetPosition);

						return robot_translation.translateBy(targetPosition.rotateBy(
								Drive.getInstance().getHeading().rotateBy(cameraYaw)));
					})
					.toList();

			List<Translation2d> positions = new ArrayList<>();

			for (Translation2d position : targetPositions) {
				if (position == null) {
					continue;
				}
				positions.add(position);
			}

			if (positions.size() == 0) {
				noteTracker.reset();
			} else {
				noteTracker.update(timestamp, positions);
			}

			for (int i = 0; i < targetPositions.size(); i++) {
				if (targetPositions.get(i) == null) {
					continue;
				}
				LogUtil.recordTranslation2d("Note " + i, targetPositions.get(i));
			}

			List<Translation2d> goalTrackerPositions = noteTracker.getTracks().stream()
					.map(report -> report.field_to_target)
					.toList();
		} else {
			lastUpdateStopwatch.startIfNotRunning();
			if (lastUpdateStopwatch.getTime() > 0.5) {
				isConnected = false;
			}
		}

		noteScan();
	}

	@Override
	public void outputTelemetry() {
		SmartDashboard.putBoolean("Has Note 1", hasNote(NotePosition.N1));
		SmartDashboard.putBoolean("Has Note 2", hasNote(NotePosition.N2));
		SmartDashboard.putBoolean("Has Note 3", hasNote(NotePosition.N3));
		SmartDashboard.putBoolean("Has Note 4", hasNote(NotePosition.N4));
		SmartDashboard.putBoolean("Has Note 5", hasNote(NotePosition.N5));
	}

	@Override
	public void stop() {}

	public void reset() {
		noteTracker.reset();
	}

	/* Logic for setting pipelines */

	public enum Pipeline {
		TELEOP(0),
		AUTO_BLUE(1),
		AUTO_RED(2);

		public final int index;

		private Pipeline(int index) {
			this.index = index;
		}
	}

	public void setPipeline(Pipeline pipeline) {
		LimelightHelpers.setPipelineIndex(name, pipeline.index);
	}

	/* Logic for matching notes */

	public enum NotePosition {
		N1(FieldLayout.kCenterNote1),
		N2(FieldLayout.kCenterNote2),
		N3(FieldLayout.kCenterNote3),
		N4(FieldLayout.kCenterNote4),
		N5(FieldLayout.kCenterNote5),
		OTHER(Translation2d.identity());

		private final Translation2d location;

		private NotePosition(Translation2d location) {
			this.location = location;
		}

		public Translation2d getLocation() {
			return location;
		}
	}
	// Finds the note that is the closest to its spot
	public NotePosition matchNotePosition(Translation2d detected_position) {
		double lowest_error = Constants.LimelightConstants.kNoteTrackEpsilon;
		NotePosition best_note_position = NotePosition.OTHER;
		for (NotePosition notePosition : NotePosition.values()) {
			double err = detected_position.distance(notePosition.getLocation());
			if (err < lowest_error) {
				best_note_position = notePosition;
				lowest_error = err;
			}
		}
		return best_note_position;
	}

	// Adds sucessful matched positions to an Arraylist
	public ArrayList<NotePosition> noteScan() {
		ArrayList<NotePosition> matchedNotes = new ArrayList<>();
		for (com.team254.lib.geometry.Translation2d detected_position : getAllNotes()) {
			NotePosition matchedPostion = matchNotePosition(detected_position);
			if (matchedPostion == NotePosition.OTHER) {
				continue;
			}
			matchedNotes.add(matchedPostion);
		}
		return matchedNotes;
	}

	/* Verifies that we are close enough to accept the match and
	checks the match is close enough to the centerline spot */
	public boolean hasNote(NotePosition note) {
		return noteScan().contains(note)
				&& Drive.getInstance()
								.getPose()
								.getTranslation()
								.minus(note.getLocation())
								.norm()
						<= Constants.LimelightConstants.kMaxNoteTrackingDistance;
	}

	public List<Translation2d> getRobotToNotes() {
		return robotToNotes;
	}

	/* Getters for having targets, notes, and limelight outputs */

	public boolean hasTarget() {
		return noteTracker.hasTracks() && getClosestNote() != null;
	}

	public Translation2d getClosestNote() {
		if (noteTracker.getTracks().get(0) != null) {
			return noteTracker.getTracks().get(0).field_to_target;
		} else {
			System.out.println("No note");
			return null;
		}
	}

	public Translation2d[] getAllNotes() {
		Translation2d[] ret = new Translation2d[noteTracker.getTracks().size()];
		for (int i = 0; i < ret.length; i++) {
			ret[i] = noteTracker.getTracks().get(i).field_to_target;
		}
		return ret;
	}

	private Translation2d getCameraToGoalTranslation(TargetInfo target, double target_height) {
		Translation2d xz_plane_translation = new Translation2d(target.getX(), target.getZ()).rotateBy(cameraPitch);
		double x = xz_plane_translation.x();
		double y = target.getY();
		double z = xz_plane_translation.y();

		// find intersection with the goal
		double differential_height = target_height - cameraHeightMeters;
		if ((z > 0.0) == (differential_height > 0.0)) {
			double scaling = differential_height / z;
			double distance = Math.hypot(x, y) * scaling;
			Rotation2d angle = new Rotation2d(x, y, true);
			return new Translation2d(distance * angle.cos(), distance * angle.sin());
		}
		return null;
	}

	private double getTotalLatencySeconds(LimelightResults results) {
		return (results.targetingResults.latency_capture
						+ results.targetingResults.latency_pipeline
						+ results.targetingResults.latency_jsonParse)
				/ 1000.0;
	}

	public boolean isConnected() {
		return isConnected;
	}

	public static class LimelightInputs {
		public double heartbeat = 0.0;
		public String jsonDump = "";
		// This should be parsed from the JSON dump input
		public LimelightResults results = new LimelightResults();
	}
}
