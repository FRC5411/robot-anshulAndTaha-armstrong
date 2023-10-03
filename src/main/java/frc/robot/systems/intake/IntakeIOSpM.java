// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.intake;

import edu.wpi.first.math.MathUtil;
import frc.robot.systems.intake.IntakeVars.Objects;

/** Class to interface with a SparkMax */
public class IntakeIOSpM implements IntakeIO {

    /** Creates a new 'program -> hardware' interface for the SparkMax */
    public IntakeIOSpM() {
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakePosition = Objects.INTAKE_ENCODER.getPosition();
        inputs.intakeVelocity = Objects.INTAKE_ENCODER.getVelocity();
        inputs.intakeTemperatureC = Objects.INTAKE_MOTOR.getMotorTemperature();
        inputs.intakeOutputCurrent = Objects.INTAKE_MOTOR.getOutputCurrent();
    }

    public void setVolts(double volts) {
        volts = MathUtil.clamp(volts, -12.0, 12.0);
        Objects.INTAKE_MOTOR.setVoltage(volts);
    }
}
