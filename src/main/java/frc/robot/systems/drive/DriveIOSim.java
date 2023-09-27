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
        inputs.leftFrontPosition = Simulation.driveSim.getLeftPositionMeters();
        inputs.leftBackPosition = Simulation.driveSim.getLeftPositionMeters();
        inputs.rightFrontPosition = Simulation.driveSim.getRightPositionMeters();
        inputs.rightFrontPosition = Simulation.driveSim.getRightPositionMeters();

        inputs.leftFrontVelocity = Simulation.driveSim.getLeftVelocityMetersPerSecond();
        inputs.leftBackVelocity = Simulation.driveSim.getLeftVelocityMetersPerSecond();
        inputs.rightFrontVelocity = Simulation.driveSim.getRightVelocityMetersPerSecond();
        inputs.rightFrontVelocity = Simulation.driveSim.getRightVelocityMetersPerSecond();
        
        inputs.leftFrontTemperatureC = 0.0;
        inputs.leftBackTemperatureC = 0.0;
        inputs.rightFrontTemperatureC = 0.0;
        inputs.rightFrontTemperatureC = 0.0;

        inputs.leftFrontAppliedCurrent = Simulation.driveSim.getLeftCurrentDrawAmps();
        inputs.leftBackAppliedCurrent = Simulation.driveSim.getLeftCurrentDrawAmps();
        inputs.rightFrontAppliedCurrent = Simulation.driveSim.getRightCurrentDrawAmps();
        inputs.rightFrontAppliedCurrent = Simulation.driveSim.getRightCurrentDrawAmps();
    }

    public void setVolts(double leftVolts, double rightVolts) {
        Simulation.driveSim.setInputs(leftVolts, rightVolts);
    }
}
