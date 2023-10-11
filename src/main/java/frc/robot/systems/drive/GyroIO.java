// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

// TODO: Change rot2d objects to doubles

/** Class to interface with the drive's gyro */
public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs {
    public Rotation2d gyroYawDeg = new Rotation2d();
    public Rotation2d gyroPitchDeg = new Rotation2d();
  }

  /** Update the Gyro's inputs */
  public default void updateInputs(GyroIOInputs inputs) {}
}
