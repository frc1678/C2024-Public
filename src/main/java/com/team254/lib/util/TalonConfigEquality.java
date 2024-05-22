package com.team254.lib.util;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;

public class TalonConfigEquality {

    public static final boolean ENABLE_LOGGING_INEQ = true;

    public static final double TALON_CONFIG_EPSILON = 0.05;

    public static boolean isEqual(TalonFXConfiguration a, TalonFXConfiguration b) {
        return isEqual(a.Slot0, b.Slot0) &&
                isEqual(a.Slot1, b.Slot1) &&
                isEqual(a.Slot2, b.Slot2) &&
                isEqual(a.MotorOutput, b.MotorOutput) &&
                isEqual(a.CurrentLimits, b.CurrentLimits) &&
                isEqual(a.Voltage, b.Voltage) &&
                isEqual(a.TorqueCurrent, b.TorqueCurrent) &&
                isEqual(a.Feedback, b.Feedback) &&
                isEqual(a.OpenLoopRamps, b.OpenLoopRamps) &&
                isEqual(a.ClosedLoopRamps, b.ClosedLoopRamps) &&
                isEqual(a.HardwareLimitSwitch, b.HardwareLimitSwitch) &&
                isEqual(a.Audio, b.Audio) &&
                isEqual(a.SoftwareLimitSwitch, b.SoftwareLimitSwitch) &&
                isEqual(a.MotionMagic, b.MotionMagic);
    }

    public static boolean isEqual(Slot0Configs a, Slot0Configs b) {
        boolean val = Util.epsilonEquals(a.kP, b.kP, TALON_CONFIG_EPSILON) &&
                Util.epsilonEquals(a.kI, b.kI, TALON_CONFIG_EPSILON) &&
                Util.epsilonEquals(a.kD, b.kD, TALON_CONFIG_EPSILON) &&
                Util.epsilonEquals(a.kV, b.kV, TALON_CONFIG_EPSILON) &&
                Util.epsilonEquals(a.kS, b.kS, TALON_CONFIG_EPSILON);
        if (ENABLE_LOGGING_INEQ && !val) {
            System.out.println("Slot0Configs not equal");
        }
        return val;
    }

    public static boolean isEqual(Slot1Configs a, Slot1Configs b) {
        boolean val = Util.epsilonEquals(a.kP, b.kP, TALON_CONFIG_EPSILON) &&
                Util.epsilonEquals(a.kI, b.kI, TALON_CONFIG_EPSILON) &&
                Util.epsilonEquals(a.kD, b.kD, TALON_CONFIG_EPSILON) &&
                Util.epsilonEquals(a.kV, b.kV, TALON_CONFIG_EPSILON) &&
                Util.epsilonEquals(a.kS, b.kS, TALON_CONFIG_EPSILON);
        if (ENABLE_LOGGING_INEQ && !val) {
            System.out.println("Slot1Configs not equal");
        }
        return val;
    }

    public static boolean isEqual(Slot2Configs a, Slot2Configs b) {
        boolean val = Util.epsilonEquals(a.kP, b.kP, TALON_CONFIG_EPSILON) &&
                Util.epsilonEquals(a.kI, b.kI, TALON_CONFIG_EPSILON) &&
                Util.epsilonEquals(a.kD, b.kD, TALON_CONFIG_EPSILON) &&
                Util.epsilonEquals(a.kV, b.kV, TALON_CONFIG_EPSILON) &&
                Util.epsilonEquals(a.kS, b.kS, TALON_CONFIG_EPSILON);
        if (ENABLE_LOGGING_INEQ && !val) {
            System.out.println("Slot2Configs not equal");
        }
        return val;
    }

    public static boolean isEqual(MotorOutputConfigs a, MotorOutputConfigs b) {
        boolean val = a.Inverted.value == b.Inverted.value
                && a.NeutralMode.value == b.NeutralMode.value
                && Util.epsilonEquals(a.DutyCycleNeutralDeadband, b.DutyCycleNeutralDeadband, TALON_CONFIG_EPSILON)
                && Util.epsilonEquals(a.PeakForwardDutyCycle, b.PeakForwardDutyCycle, TALON_CONFIG_EPSILON)
                && Util.epsilonEquals(a.PeakReverseDutyCycle, b.PeakReverseDutyCycle, TALON_CONFIG_EPSILON);
        if (ENABLE_LOGGING_INEQ && !val) {
            System.out.println("MotorOutputConfigs not equal");
        }
        return val;
    }

    public static boolean isEqual(CurrentLimitsConfigs a, CurrentLimitsConfigs b) {
        boolean val = Util.epsilonEquals(a.StatorCurrentLimit, b.StatorCurrentLimit, TALON_CONFIG_EPSILON)
                && Util.epsilonEquals(a.SupplyCurrentLimit, b.SupplyCurrentLimit, TALON_CONFIG_EPSILON)
                && a.StatorCurrentLimitEnable == b.StatorCurrentLimitEnable
                && a.SupplyCurrentLimitEnable == b.SupplyCurrentLimitEnable;
        if (ENABLE_LOGGING_INEQ && !val) {
            System.out.println("CurrentLimitsConfigs not equal");
        }
        return val;
    }

    public static boolean isEqual(VoltageConfigs a, VoltageConfigs b) {
        boolean val = Util.epsilonEquals(a.SupplyVoltageTimeConstant, b.SupplyVoltageTimeConstant, TALON_CONFIG_EPSILON)
                && Util.epsilonEquals(a.PeakForwardVoltage, b.PeakForwardVoltage)
                && Util.epsilonEquals(a.PeakReverseVoltage, b.PeakReverseVoltage);
        if (ENABLE_LOGGING_INEQ && !val) {
            System.out.println("VoltageConfigs not equal");
        }
        return val;
    }

    public static boolean isEqual(TorqueCurrentConfigs a, TorqueCurrentConfigs b) {
        boolean val = Util.epsilonEquals(a.PeakForwardTorqueCurrent, b.PeakForwardTorqueCurrent, TALON_CONFIG_EPSILON)
                && Util.epsilonEquals(a.PeakReverseTorqueCurrent, b.PeakReverseTorqueCurrent, TALON_CONFIG_EPSILON)
                && Util.epsilonEquals(a.TorqueNeutralDeadband, b.TorqueNeutralDeadband, TALON_CONFIG_EPSILON);
        if (ENABLE_LOGGING_INEQ && !val) {
            System.out.println("TorqueCurrentConfigs not equal");
        }
        return val;
    }

    public static boolean isEqual(FeedbackConfigs a, FeedbackConfigs b) {
        boolean val = Util.epsilonEquals(a.FeedbackRotorOffset, b.FeedbackRotorOffset, TALON_CONFIG_EPSILON)
                && Util.epsilonEquals(a.SensorToMechanismRatio, b.SensorToMechanismRatio, TALON_CONFIG_EPSILON)
                && Util.epsilonEquals(a.RotorToSensorRatio, b.RotorToSensorRatio, TALON_CONFIG_EPSILON)
                && a.FeedbackSensorSource.value == b.FeedbackSensorSource.value
                && a.FeedbackRemoteSensorID == b.FeedbackRemoteSensorID;
        if (ENABLE_LOGGING_INEQ && !val) {
            System.out.println("FeedbackConfigs not equal");
            System.out.printf("A-FeedbackRotorOffset: %f, B-FeedbackRotorOffset: %f\n", a.FeedbackRotorOffset,
                    b.FeedbackRotorOffset);
            System.out.printf("A-SensorToMechanismRatio: %f, B-SensorToMechanismRatio: %f\n", a.SensorToMechanismRatio,
                    b.SensorToMechanismRatio);
            System.out.printf("A-RotorToSensorRatio: %f, B-RotorToSensorRatio: %f\n", a.RotorToSensorRatio,
                    b.RotorToSensorRatio);
            System.out.printf("A-FeedbackSensorSource: %d, B-FeedbackSensorSource: %d\n", a.FeedbackSensorSource.value,
                    b.FeedbackSensorSource.value);
            System.out.printf("A-FeedbackRemoteSensorID: %d, B-FeedbackRemoteSensorID: %d\n", a.FeedbackRemoteSensorID,
                    b.FeedbackRemoteSensorID);

        }
        return val;
    }

    public static boolean isEqual(OpenLoopRampsConfigs a, OpenLoopRampsConfigs b) {
        boolean val = Util.epsilonEquals(a.DutyCycleOpenLoopRampPeriod, b.DutyCycleOpenLoopRampPeriod,
                TALON_CONFIG_EPSILON)
                && Util.epsilonEquals(a.VoltageOpenLoopRampPeriod, b.VoltageOpenLoopRampPeriod, TALON_CONFIG_EPSILON)
                && Util.epsilonEquals(a.TorqueOpenLoopRampPeriod, b.TorqueOpenLoopRampPeriod, TALON_CONFIG_EPSILON);
        if (ENABLE_LOGGING_INEQ && !val) {
            System.out.println("OpenLoopRampsConfigs not equal");
        }
        return val;
    }

    public static boolean isEqual(ClosedLoopRampsConfigs a, ClosedLoopRampsConfigs b) {
        boolean val = Util.epsilonEquals(a.DutyCycleClosedLoopRampPeriod, b.DutyCycleClosedLoopRampPeriod,
                TALON_CONFIG_EPSILON)
                && Util.epsilonEquals(a.VoltageClosedLoopRampPeriod, b.VoltageClosedLoopRampPeriod,
                        TALON_CONFIG_EPSILON)
                && Util.epsilonEquals(a.TorqueClosedLoopRampPeriod, b.TorqueClosedLoopRampPeriod, TALON_CONFIG_EPSILON);
        if (ENABLE_LOGGING_INEQ && !val) {
            System.out.println("ClosedLoopRampsConfigs not equal");
        }
        return val;
    }

    public static boolean isEqual(HardwareLimitSwitchConfigs a, HardwareLimitSwitchConfigs b) {
        boolean val = a.ForwardLimitAutosetPositionEnable == b.ForwardLimitAutosetPositionEnable
                && b.ForwardLimitEnable == b.ForwardLimitEnable
                && a.ReverseLimitAutosetPositionEnable == b.ReverseLimitAutosetPositionEnable
                && a.ReverseLimitEnable == b.ReverseLimitEnable
                && Util.epsilonEquals(a.ForwardLimitAutosetPositionValue, b.ForwardLimitAutosetPositionValue,
                        TALON_CONFIG_EPSILON)
                && Util.epsilonEquals(a.ReverseLimitAutosetPositionValue, b.ReverseLimitAutosetPositionValue,
                        TALON_CONFIG_EPSILON)
                && a.ForwardLimitRemoteSensorID == b.ForwardLimitRemoteSensorID
                && a.ReverseLimitRemoteSensorID == b.ReverseLimitRemoteSensorID
                && a.ForwardLimitSource.value == b.ForwardLimitSource.value
                && a.ReverseLimitSource.value == b.ReverseLimitSource.value
                && a.ForwardLimitType.value == b.ForwardLimitType.value
                && a.ReverseLimitType.value == b.ReverseLimitType.value;
        if (ENABLE_LOGGING_INEQ && !val) {
            System.out.println("HardwareLimitSwitchConfigs not equal");
        }
        return val;
    }

    public static boolean isEqual(AudioConfigs a, AudioConfigs b) {
        boolean val = a.BeepOnBoot == b.BeepOnBoot;
        if (ENABLE_LOGGING_INEQ && !val) {
            System.out.println("AudioConfigs not equal");
        }
        return val;
    }

    public static boolean isEqual(SoftwareLimitSwitchConfigs a, SoftwareLimitSwitchConfigs b) {
        boolean val = Util.epsilonEquals(a.ForwardSoftLimitThreshold, b.ForwardSoftLimitThreshold, TALON_CONFIG_EPSILON)
                && Util.epsilonEquals(a.ReverseSoftLimitThreshold, b.ReverseSoftLimitThreshold, TALON_CONFIG_EPSILON)
                && a.ReverseSoftLimitEnable == b.ReverseSoftLimitEnable
                && a.ForwardSoftLimitEnable == b.ForwardSoftLimitEnable;
        if (ENABLE_LOGGING_INEQ && !val) {
            System.out.println("SoftwareLimitSwitchConfigs not equal");
        }
        return val;
    }

    public static boolean isEqual(MotionMagicConfigs a, MotionMagicConfigs b) {
        boolean val = Util.epsilonEquals(a.MotionMagicAcceleration, b.MotionMagicAcceleration, TALON_CONFIG_EPSILON)
                && Util.epsilonEquals(a.MotionMagicCruiseVelocity, b.MotionMagicCruiseVelocity, TALON_CONFIG_EPSILON)
                && Util.epsilonEquals(a.MotionMagicJerk, b.MotionMagicJerk, TALON_CONFIG_EPSILON);
        if (ENABLE_LOGGING_INEQ && !val) {
            System.out.println("MotionMagicConfigs not equal");
        }
        return val;
    }
}
