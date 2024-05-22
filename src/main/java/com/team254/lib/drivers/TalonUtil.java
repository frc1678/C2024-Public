package com.team254.lib.drivers;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team254.lib.util.TalonConfigEquality;

import edu.wpi.first.wpilibj.DriverStation;


public class TalonUtil {

    public static boolean applyAndCheckConfiguration(TalonFX talon, TalonFXConfiguration config, int numTries) {
        for (int i = 0; i < numTries; i++) {
            if (Phoenix6Util.checkErrorAndRetry(() -> talon.getConfigurator().apply(config))) {
                // API says we applied config, lets make sure it's right
                if (readAndVerifyConfiguration(talon, config)) {
                    return true;
                } else {
                    DriverStation.reportWarning("Failed to verify config for talon [" + talon.getDescription()
                            + "] (attempt " + (i + 1) + " of " + numTries + ")", false);
                }
            } else {
                DriverStation.reportWarning("Failed to apply config for talon [" + talon.getDescription()
                        + "] (attempt " + (i + 1) + " of " + numTries + ")", false);
            }
        }
        DriverStation.reportError("Failed to apply config for talon after " + numTries + " attempts", false);
        return false;
    }

    public static boolean readAndVerifyConfiguration(TalonFX talon, TalonFXConfiguration config) {
        TalonFXConfiguration readConfig = new TalonFXConfiguration();
        if (!Phoenix6Util.checkErrorAndRetry(() -> talon.getConfigurator().refresh(readConfig))) {
            // could not get config!
            DriverStation.reportWarning("Failed to read config for talon [" + talon.getDescription() + "]", false);
            return false;
        } else if (!TalonConfigEquality.isEqual(config, readConfig)) {
            // configs did not match
            DriverStation.reportWarning("Configuration verification failed for talon [" + talon.getDescription() + "]",
                    false);
            return false;
        } else {
            // configs read and match, Talon OK
            return true;
        }
    }

    public static boolean applyAndCheckConfiguration(TalonFX talon, TalonFXConfiguration config) {
        boolean result = applyAndCheckConfiguration(talon, config, 2);
        return result;
    }

    public enum StickyFault {
        BootDuringEnable,
        DeviceTemp,
        ForwardHardLimit,
        ForwardSoftLimit,
        Hardware,
        OverSupplyV,
        ProcTemp,
        ReverseHardLimit,
        ReverseSoftLimit,
        Undervoltage,
        UnstableSupplyV
    }

    public static void checkStickyFaults(String subsystemName, TalonFX talon) {
        boolean[] faults = new boolean[StickyFault.values().length];
        faults[0] = talon.getStickyFault_BootDuringEnable().getValue();
        faults[1] = talon.getStickyFault_DeviceTemp().getValue();
        faults[2] = talon.getStickyFault_ForwardHardLimit().getValue();
        faults[3] = talon.getStickyFault_ForwardSoftLimit().getValue();
        faults[4] = talon.getStickyFault_Hardware().getValue();
        faults[5] = talon.getStickyFault_OverSupplyV().getValue();
        faults[6] = talon.getStickyFault_ProcTemp().getValue();
        faults[7] = talon.getStickyFault_ReverseHardLimit().getValue();
        faults[8] = talon.getStickyFault_ReverseSoftLimit().getValue();
        faults[9] = talon.getStickyFault_Undervoltage().getValue();
        faults[10] = talon.getStickyFault_UnstableSupplyV().getValue();

        for (int i = 0; i < faults.length; i++) {
            if (faults[i]) {
                DriverStation.reportError(subsystemName + ": Talon Fault! " + StickyFault.values()[i].toString(),
                        false);
            }
        }

        talon.clearStickyFaults();
    }
}
