// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.drive;

import org.littletonrobotics.junction.AutoLog;

/** Class to interface with the drive subsytem */
public interface DriveIO {
    @AutoLog
    public static class DriveIOInputs {
        public double leftFrontPosition = 0.0;
        public double leftBackPosition = 0.0;
        public double rightFrontPosition = 0.0;
        public double rightBackPosition = 0.0;

        public double leftFrontVelocity = 0.0;
        public double leftBackVelocity = 0.0;
        public double rightFrontVelocity = 0.0;
        public double rightBackVelocity = 0.0;

        public double leftFrontTemperatureC = 0.0;
        public double leftBackTemperatureC = 0.0;
        public double rightFrontTemperatureC = 0.0;
        public double rightBackTemperatureC = 0.0;

        public double leftFrontAppliedCurrent = 0.0;
        public double leftBackAppliedCurrent = 0.0;
        public double rightFrontAppliedCurrent = 0.0;
        public double rightBackAppliedCurrent = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(DriveIOInputs inputs) {
    }

    /** Sets the voltage of the left and right side of the drivetrain */
    public default void setVolts(double leftVolts, double rightVolts) {
    }
}
