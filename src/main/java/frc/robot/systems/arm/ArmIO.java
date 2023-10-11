// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.arm;

import org.littletonrobotics.junction.AutoLog;

/** Class to interface with the drive subsytem */
public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public double armPosition = 0.0;
    public double armVelocity = 0.0;
    public double armTemperatureC = 0.0;
    public double armAppliedCurrent = 0.0;
  }

  /** Update the set of loggable inputs */
  public default void updateInputs(ArmIOInputs inputs) {}

  /** Sets the voltage of the arm motor */
  public default void setVolts(double volts) {}
}
