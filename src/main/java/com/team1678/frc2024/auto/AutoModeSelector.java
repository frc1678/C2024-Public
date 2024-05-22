package com.team1678.frc2024.auto;

import com.team1678.frc2024.auto.modes.*;
import com.team1678.frc2024.auto.modes.adaptive.AdaptiveSixMode;
import com.team1678.frc2024.auto.modes.fiveline.N4FiveLineMode;
import com.team1678.frc2024.auto.modes.fiveline.N5FiveLineMode;
import com.team1678.frc2024.auto.modes.legacy.AlternateSixNote;
import com.team1678.frc2024.auto.modes.legacy.ShootAndDriveMode;
import com.team1678.frc2024.auto.modes.legacy.SixNoteMode;
import com.team1678.frc2024.auto.modes.sfsix.OneThreeSemiFastSixNoteMode;
import com.team1678.frc2024.auto.modes.sfsix.SemiFastSixNoteMode;
import com.team1678.frc2024.auto.modes.three.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;

public class AutoModeSelector {
	public enum DesiredMode {
		DO_NOTHING,
		TEST_PATH_AUTO,
		SIX_NOTE_MODE,
		ALTERNATE_SIX_NOTE_MODE,
		SEMI_FAST_SIX_NOTE_MODE,
		ONE_THREE_SEMI_FAST_SIX_NOTE_MODE,
		SHOOT_AND_DRIVE,
		THREE_NOTE_MODE_54,
		THREE_NOTE_MODE_53,
		THREE_NOTE_MODE_45,
		THREE_NOTE_MODE_43,
		THREE_NOTE_MODE_35,
		THREE_NOTE_MODE_34,
		RIGHT_FIVE_MODE_4,
		RIGHT_FIVE_MODE_5,
		ADAPTIVE_SIX_MODE
	}

	public enum TargetNote {
		N1,
		N2,
		N3,
		N4,
		N5
	}

	public enum TargetSpike {
		LEFT,
		CENTER,
		RIGHT
	}

	private DesiredMode mCachedDesiredMode = DesiredMode.DO_NOTHING;
	private TargetNote mCachedFirstNote = TargetNote.N1;
	private TargetNote mCachedSecondNote = TargetNote.N2;
	private TargetSpike mCachedSpike = TargetSpike.LEFT;

	private Optional<AutoModeBase> mAutoMode = Optional.empty();

	private static SendableChooser<DesiredMode> mModeChooser = new SendableChooser<>();
	private static SendableChooser<TargetNote> mFirstNoteChooser = new SendableChooser<>();
	private static SendableChooser<TargetNote> mSecondNoteChooser = new SendableChooser<>();
	private static SendableChooser<TargetSpike> mSpikeChooser = new SendableChooser<>();

	public AutoModeSelector() {
		mModeChooser.addOption("Do Nothing", DesiredMode.DO_NOTHING);
		mModeChooser.setDefaultOption("Semi Fast Six Note", DesiredMode.SEMI_FAST_SIX_NOTE_MODE);
		mModeChooser.setDefaultOption("13 Semi Fast Six Note", DesiredMode.ONE_THREE_SEMI_FAST_SIX_NOTE_MODE);
		mModeChooser.addOption("Three Note 4/5 + Limelight", DesiredMode.THREE_NOTE_MODE_45);
		mModeChooser.addOption("TESTPATH!!!", DesiredMode.TEST_PATH_AUTO);
		mModeChooser.addOption("SHOOT AND DRIVE", DesiredMode.SHOOT_AND_DRIVE);
		mModeChooser.addOption("Six Note", DesiredMode.SIX_NOTE_MODE);
		mModeChooser.addOption("Alternate Six Note", DesiredMode.ALTERNATE_SIX_NOTE_MODE);
		mModeChooser.addOption("Three Note 5/4", DesiredMode.THREE_NOTE_MODE_54);
		mModeChooser.addOption("Three Note 5/3", DesiredMode.THREE_NOTE_MODE_53);
		mModeChooser.addOption("Three Note 4/3", DesiredMode.THREE_NOTE_MODE_43);
		mModeChooser.addOption("Three Note 3/5 + Limelight", DesiredMode.THREE_NOTE_MODE_35);
		mModeChooser.addOption("Three Note 3/4", DesiredMode.THREE_NOTE_MODE_34);
		mModeChooser.addOption("Adaptive Six", DesiredMode.ADAPTIVE_SIX_MODE);
		mModeChooser.addOption("N4 Source Five Mode", DesiredMode.RIGHT_FIVE_MODE_4);
		mModeChooser.addOption("N5 Source Five Mode", DesiredMode.RIGHT_FIVE_MODE_5);

		SmartDashboard.putData("Auto Mode", mModeChooser);

		mFirstNoteChooser.setDefaultOption("N1", TargetNote.N1);
		mFirstNoteChooser.addOption("N2", TargetNote.N2);
		mFirstNoteChooser.addOption("N3", TargetNote.N3);
		mFirstNoteChooser.addOption("N4", TargetNote.N4);
		mFirstNoteChooser.addOption("N5", TargetNote.N5);
		SmartDashboard.putData("First Note", mFirstNoteChooser);

		mSecondNoteChooser.addOption("N1", TargetNote.N1);
		mSecondNoteChooser.setDefaultOption("N2", TargetNote.N2);
		mSecondNoteChooser.addOption("N3", TargetNote.N3);
		mSecondNoteChooser.addOption("N4", TargetNote.N4);
		mSecondNoteChooser.addOption("N5", TargetNote.N5);
		SmartDashboard.putData("Second Note", mSecondNoteChooser);

		mSpikeChooser.setDefaultOption("Left", TargetSpike.LEFT);
		mSpikeChooser.addOption("Center", TargetSpike.CENTER);
		mSpikeChooser.addOption("Right", TargetSpike.RIGHT);
		SmartDashboard.putData("First Spike", mSpikeChooser);
	}

	public void updateModeCreator(boolean force_regen) {
		DesiredMode desiredMode = mModeChooser.getSelected();
		TargetNote firstNote = mFirstNoteChooser.getSelected();
		TargetNote secondNote = mSecondNoteChooser.getSelected();
		TargetSpike desiredSpike = mSpikeChooser.getSelected();

		if (desiredMode == null) {
			desiredMode = DesiredMode.DO_NOTHING;
		}
		if (mCachedDesiredMode != desiredMode
				|| mCachedFirstNote != firstNote
				|| mCachedSecondNote != secondNote
				|| mCachedSpike != desiredSpike
				|| force_regen) {
			System.out.println("Auto selection changed, updating creator: desiredMode-> " + desiredMode.name() + "//"
					+ firstNote.name() + "//" + secondNote.name() + "//" + desiredMode.name() + " Spike");
			mAutoMode = getAutoModeForParams(desiredMode, firstNote, secondNote, desiredSpike);
		}
		mCachedDesiredMode = desiredMode;
		mCachedFirstNote = firstNote;
		mCachedSecondNote = secondNote;
		mCachedSpike = desiredSpike;
	}

	private Optional<AutoModeBase> getAutoModeForParams(
			DesiredMode mode, TargetNote n_0, TargetNote n_1, TargetSpike s_0) {
		switch (mode) {
			case DO_NOTHING:
				return Optional.of(new DoNothingMode());

			case TEST_PATH_AUTO:
				return Optional.of(new TestPathMode());

			case SIX_NOTE_MODE:
				return Optional.of(new SixNoteMode());

			case ALTERNATE_SIX_NOTE_MODE:
				return Optional.of(new AlternateSixNote());

			case SEMI_FAST_SIX_NOTE_MODE:
				return Optional.of(new SemiFastSixNoteMode());

			case ONE_THREE_SEMI_FAST_SIX_NOTE_MODE:
				return Optional.of(new OneThreeSemiFastSixNoteMode());

			case ADAPTIVE_SIX_MODE:
				return Optional.of(new AdaptiveSixMode(n_0, n_1, s_0));

			case SHOOT_AND_DRIVE:
				return Optional.of(new ShootAndDriveMode());

			case THREE_NOTE_MODE_54:
				return Optional.of(new ThreeNoteMode54());

			case THREE_NOTE_MODE_53:
				return Optional.of(new ThreeNoteMode53());

			case THREE_NOTE_MODE_45:
				return Optional.of(new ThreeNoteMode45());

			case THREE_NOTE_MODE_43:
				return Optional.of(new ThreeNoteMode43());

			case THREE_NOTE_MODE_35:
				return Optional.of(new ThreeNoteMode35());

			case THREE_NOTE_MODE_34:
				return Optional.of(new ThreeNoteMode34());

			case RIGHT_FIVE_MODE_4:
				return Optional.of(new N4FiveLineMode());

			case RIGHT_FIVE_MODE_5:
				return Optional.of(new N5FiveLineMode());

			default:
				System.out.println("ERROR: unexpected auto mode: " + mode);
				break;
		}

		System.err.println("No valid auto mode found for  " + mode);
		return Optional.empty();
	}

	public static SendableChooser<DesiredMode> getModeChooser() {
		return mModeChooser;
	}

	public DesiredMode getDesiredAutomode() {
		return mCachedDesiredMode;
	}

	public void reset() {
		mAutoMode = Optional.empty();
		mCachedDesiredMode = null;
	}

	public void outputToSmartDashboard() {
		SmartDashboard.putString("AutoModeSelected", mCachedDesiredMode.name());
	}

	public Optional<AutoModeBase> getAutoMode() {
		if (!mAutoMode.isPresent()) {
			return Optional.empty();
		}
		return mAutoMode;
	}

	public boolean isDriveByCamera() {
		return mCachedDesiredMode == DesiredMode.DO_NOTHING;
	}
}
