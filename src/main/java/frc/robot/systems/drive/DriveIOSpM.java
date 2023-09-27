// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.drive;

import frc.robot.systems.drive.DriveVars.Objects;

/** Class to interface with a SparkMax */
public class DriveIOSpM implements DriveIO {

    /** Creates a new 'program -> hardware' interface for the SparkMax */
    public DriveIOSpM() {}

    @Override
    public void updateInputs(DriveIOInputs inputs) {
        inputs.leftFrontPosition = Objects.leftFrontEncoder.getPosition();
        inputs.leftBackPosition = Objects.leftBackEncoder.getPosition();
        inputs.rightFrontPosition = Objects.rightFrontEncoder.getPosition();
        inputs.rightFrontPosition = Objects.rightFrontEncoder.getPosition();

        inputs.leftFrontVelocity = Objects.leftFrontEncoder.getVelocity();
        inputs.leftBackVelocity = Objects.leftBackEncoder.getVelocity();
        inputs.rightFrontVelocity = Objects.rightFrontEncoder.getVelocity();
        inputs.rightFrontVelocity = Objects.rightFrontEncoder.getVelocity();
        
        inputs.leftFrontTemperatureC = Objects.leftFront.getMotorTemperature();
        inputs.leftBackTemperatureC = Objects.leftBack.getMotorTemperature();
        inputs.rightFrontTemperatureC = Objects.rightFront.getMotorTemperature();
        inputs.rightFrontTemperatureC = Objects.rightFront.getMotorTemperature();

        inputs.leftFrontAppliedCurrent = Objects.leftFront.getOutputCurrent();
        inputs.leftBackAppliedCurrent = Objects.leftBack.getOutputCurrent();
        inputs.rightFrontAppliedCurrent = Objects.rightFront.getOutputCurrent();
        inputs.rightFrontAppliedCurrent = Objects.rightFront.getOutputCurrent();
    }

    public void setVolts(double leftVolts, double rightVolts) {
        Objects.leftFront.setVoltage(leftVolts);
        Objects.leftBack.setVoltage(leftVolts);
        Objects.rightFront.setVoltage(rightVolts);
        Objects.rightBack.setVoltage(rightVolts);
    }
}
