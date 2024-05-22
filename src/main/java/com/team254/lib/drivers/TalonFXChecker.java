package com.team254.lib.drivers;

import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team1678.frc2024.subsystems.Subsystem;

import java.util.ArrayList;

public class TalonFXChecker extends MotorChecker<TalonFX> {
    private static class ControlRequest {
        public StatusSignal<ControlModeValue> mMode;
        public double mSetValue;
    }

    protected ArrayList<ControlRequest> mControlRequests = new ArrayList<>();

    public static boolean checkMotors(Subsystem subsystem,
                                      ArrayList<MotorConfig<TalonFX>> motorsToCheck,
                                      CheckerConfig checkerConfig) {
        TalonFXChecker checker = new TalonFXChecker();
        return checker.checkMotorsImpl(subsystem, motorsToCheck, checkerConfig);
    }

    @Override
    protected void storeConfiguration() {
        // record previous configuration for all talons
        for (MotorConfig<TalonFX> config : mMotorsToCheck) {
            TalonFX talon = (TalonFX) config.mMotor;

            ControlRequest configuration = new ControlRequest();
            configuration.mMode = talon.getControlMode();

            mControlRequests.add(configuration);
        }
    }

    @Override
    protected void restoreConfiguration() {
        for (int i = 0; i < mMotorsToCheck.size(); ++i) {
            mMotorsToCheck.get(i).mMotor.getAppliedControl();
            mMotorsToCheck.get(i).mMotor.set(mControlRequests.get(i).mSetValue);
        }
    }

    @Override
    protected void setMotorOutput(TalonFX motor, double output) {
        motor.set(output);
    }

    @Override
    public double getMotorCurrent(TalonFX motor) {
        return motor.getStatorCurrent().getValue();
    }

}