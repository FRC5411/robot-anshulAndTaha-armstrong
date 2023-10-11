// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.intake;

import org.littletonrobotics.junction.AutoLog;

/** Class to interface with the intake subsytem */
public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double intakePosition = 0.0;
    public double intakeVelocity = 0.0;
    public double intakeTemperatureC = 0.0;
    public double intakeOutputCurrent = 0.0;
  }

  /** Update the set of loggable inputs */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Sets the voltage of the intake motor */
  public default void setVolts(double volts) {}
}
