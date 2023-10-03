// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.intake;

import edu.wpi.first.math.MathUtil;

/** Simulates the intake (Neo550) motor */
public class IntakeIOSim implements IntakeIO {

    // NOTE: This class does not do anything

    /** Creates a new simulation for the intake */
    public IntakeIOSim() {
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakePosition = 0.0;
        inputs.intakeVelocity = 0.0;
        inputs.intakeTemperatureC = 0.0;
        inputs.intakeOutputCurrent = 0.0;
    }

    public void setVolts(double volts) {
        volts = MathUtil.clamp(volts, -12.0, 12.0);
    }
}
