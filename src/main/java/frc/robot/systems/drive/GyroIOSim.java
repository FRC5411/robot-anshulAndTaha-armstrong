// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.systems.drive.DriveVars.Simulation;

/** Class to interface with a Gyro Sim */
public class GyroIOSim implements GyroIO {

  /** Creates a new interface for the Sim */
  public GyroIOSim() {}

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.gyroYawDeg = Simulation.DRIVE_SIMULATOR.getHeading();
    inputs.gyroPitchDeg = new Rotation2d();
  }
}
