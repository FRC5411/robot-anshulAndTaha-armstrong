// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.arm;

import edu.wpi.first.math.MathUtil;
import frc.robot.systems.arm.ArmVars.Objects;

/** Class to interface with a SparkMax */
public class ArmIOSpM implements ArmIO {

  /** Creates a new 'program -> hardware' interface for the SparkMax */
  public ArmIOSpM() {}

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.armPosition = Objects.ARM_ENCODER.getDistance();
    inputs.armVelocity = Objects.ARM_ENCODER.getRate();
    inputs.armTemperatureC = Objects.ARM_MOTOR.getMotorTemperature();
    inputs.armAppliedCurrent = Objects.ARM_MOTOR.getOutputCurrent();
  }

  public void setVolts(double volts) {
    volts = MathUtil.clamp(volts, -12.0, 12.0);
    Objects.ARM_MOTOR.setVoltage(volts);
  }
}
