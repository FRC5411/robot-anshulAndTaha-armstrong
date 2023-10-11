// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.drive;

import frc.robot.systems.drive.DriveVars.Simulation;

/** Simulates a drivtrain */
public class DriveIOSim implements DriveIO {

  /** Creates a new simulation for the drivetrain */
  public DriveIOSim() {}

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    inputs.leftFrontPosition = Simulation.DRIVE_SIMULATOR.getLeftPositionMeters();
    inputs.leftBackPosition = Simulation.DRIVE_SIMULATOR.getLeftPositionMeters();
    inputs.rightFrontPosition = Simulation.DRIVE_SIMULATOR.getRightPositionMeters();
    inputs.rightBackPosition = Simulation.DRIVE_SIMULATOR.getRightPositionMeters();

    inputs.leftFrontVelocity = Simulation.DRIVE_SIMULATOR.getLeftVelocityMetersPerSecond();
    inputs.leftBackVelocity = Simulation.DRIVE_SIMULATOR.getLeftVelocityMetersPerSecond();
    inputs.rightFrontVelocity = Simulation.DRIVE_SIMULATOR.getRightVelocityMetersPerSecond();
    inputs.rightBackVelocity = Simulation.DRIVE_SIMULATOR.getRightVelocityMetersPerSecond();

    inputs.leftFrontTemperatureC = 0.0;
    inputs.leftBackTemperatureC = 0.0;
    inputs.rightFrontTemperatureC = 0.0;
    inputs.rightBackTemperatureC = 0.0;

    inputs.leftFrontAppliedCurrent = Simulation.DRIVE_SIMULATOR.getLeftCurrentDrawAmps();
    inputs.leftBackAppliedCurrent = Simulation.DRIVE_SIMULATOR.getLeftCurrentDrawAmps();
    inputs.rightFrontAppliedCurrent = Simulation.DRIVE_SIMULATOR.getRightCurrentDrawAmps();
    inputs.rightBackAppliedCurrent = Simulation.DRIVE_SIMULATOR.getRightCurrentDrawAmps();
  }

  public void setVolts(double leftVolts, double rightVolts) {
    Simulation.DRIVE_SIMULATOR.setInputs(leftVolts, rightVolts);
  }
}
