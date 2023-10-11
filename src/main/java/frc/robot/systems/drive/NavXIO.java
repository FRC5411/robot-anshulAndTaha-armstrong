// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.systems.drive.DriveVars.Objects;

/** Class to interface with a NavX */
public class NavXIO implements GyroIO {

  /** Creates a new interface for the NavX */
  public NavXIO() {}

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.gyroYawDeg = Rotation2d.fromDegrees(Objects.NAVX.getYaw());
    inputs.gyroPitchDeg = Rotation2d.fromDegrees(Objects.NAVX.getPitch());
  }
}
