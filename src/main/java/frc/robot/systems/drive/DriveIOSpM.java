// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.drive;

import frc.robot.systems.drive.DriveVars.Objects;

/** Class to interface with a SparkMax */
public class DriveIOSpM implements DriveIO {

    /** Creates a new 'program -> hardware' interface for the SparkMax */
    public DriveIOSpM() {
    }

    @Override
    public void updateInputs(DriveIOInputs inputs) {
        inputs.leftFrontPosition = Objects.LEFT_FRONT_ENCODER.getPosition();
        inputs.leftBackPosition = Objects.LEFT_BACK_ENCODER.getPosition();
        inputs.rightFrontPosition = Objects.RIGHT_FRONT_ENCODER.getPosition();
        inputs.rightBackPosition = Objects.RIGHT_BACK_ENCODER.getPosition();

        inputs.leftFrontVelocity = Objects.LEFT_FRONT_ENCODER.getVelocity();
        inputs.leftBackVelocity = Objects.LEFT_BACK_ENCODER.getVelocity();
        inputs.rightFrontVelocity = Objects.RIGHT_FRONT_ENCODER.getVelocity();
        inputs.rightBackVelocity = Objects.RIGHT_BACK_ENCODER.getVelocity();

        inputs.leftFrontTemperatureC = Objects.LEFT_FRONT_MOTOR.getMotorTemperature();
        inputs.leftBackTemperatureC = Objects.LEFT_BACK_MOTOR.getMotorTemperature();
        inputs.rightFrontTemperatureC = Objects.RIGHT_FRONT_MOTOR.getMotorTemperature();
        inputs.rightBackTemperatureC = Objects.RIGHT_BACK_MOTOR.getMotorTemperature();

        inputs.leftFrontAppliedCurrent = Objects.LEFT_FRONT_MOTOR.getOutputCurrent();
        inputs.leftBackAppliedCurrent = Objects.LEFT_BACK_MOTOR.getOutputCurrent();
        inputs.rightFrontAppliedCurrent = Objects.RIGHT_FRONT_MOTOR.getOutputCurrent();
        inputs.rightBackAppliedCurrent = Objects.RIGHT_BACK_MOTOR.getOutputCurrent();
    }

    public void setVolts(double leftVolts, double rightVolts) {
        Objects.LEFT_FRONT_MOTOR.setVoltage(leftVolts);
        Objects.LEFT_BACK_MOTOR.setVoltage(leftVolts);
        Objects.RIGHT_FRONT_MOTOR.setVoltage(rightVolts);
        Objects.RIGHT_BACK_MOTOR.setVoltage(rightVolts);
    }
}
