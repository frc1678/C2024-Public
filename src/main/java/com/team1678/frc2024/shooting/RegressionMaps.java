package com.team1678.frc2024.shooting;

import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.InterpolatingTreeMap;

public class RegressionMaps {
	public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kHoodAutoAimMap =
			new InterpolatingTreeMap<>();

	static {
		for (double[] pair : SpeakerRegression.kHoodManualAngle) {
			kHoodAutoAimMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
		}
	}

	public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kFlywheelAutoAimMap =
			new InterpolatingTreeMap<>();

	static {
		for (double[] pair : SpeakerRegression.kFlywheelManualRPM) {
			kFlywheelAutoAimMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
		}
	}

	public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kShooterCurveOffsetMap =
			new InterpolatingTreeMap<>();

	static {
		for (double[] pair : SpeakerRegression.kShooterCurveOffset) {
			kShooterCurveOffsetMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
		}
	}

	public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kSkewOffsetMap =
			new InterpolatingTreeMap<>();

	static {
		for (double[] pair : SpeakerRegression.kSkewOffset) {
			kSkewOffsetMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
		}
	}

	public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kLowFerryHoodMap =
			new InterpolatingTreeMap<>();

	static {
		for (double[] pair : FerryRegression.kLowFerryAngle) {
			kLowFerryHoodMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
		}
	}

	public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kLowFerryRPMMap =
			new InterpolatingTreeMap<>();

	static {
		for (double[] pair : FerryRegression.kLowFerryRPM) {
			kLowFerryRPMMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
		}
	}

	public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kHighFerryHoodMap =
			new InterpolatingTreeMap<>();

	static {
		for (double[] pair : FerryRegression.kHighFerryAngle) {
			kHighFerryHoodMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
		}
	}

	public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kHighFerryRPMMap =
			new InterpolatingTreeMap<>();

	static {
		for (double[] pair : FerryRegression.kHighFerryRPM) {
			kHighFerryRPMMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
		}
	}
}
