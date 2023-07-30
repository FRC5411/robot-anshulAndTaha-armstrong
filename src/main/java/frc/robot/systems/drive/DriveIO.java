// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.drive;

import java.util.HashMap;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.systems.drive.DriveVars.Constants;
import frc.robot.systems.drive.DriveVars.Objects;

/** Add your docs here. */
public class DriveIO {
    public DriveIO() {}

    ////// TELEOP \\\\\\\
    public void arcadeDrive(double speed, double rotation) {
        Objects.robotDrive.arcadeDrive(speed, rotation);
    }

    public void teleopDrive(double speed, double rotation) {
        rotation *= Constants.kRotationScaler;

        if (Math.abs(speed) < Constants.kDeadzone) {
            speed = 0;
        }

        if (Math.abs(rotation) < Constants.kDeadzone) {
            rotation = 0;
        }

        Objects.robotDrive.arcadeDrive(speed, rotation);
    }

    /////// PATH PLANNER \\\\\\\
    public Command followPathwithEvents(PathPlannerTrajectory traj, boolean useColor, HashMap<String, Command> eventMap, Subsystem drive) {
        RamseteAutoBuilder builder = new RamseteAutoBuilder(
            this::getPose, 
            this::resetPose, 
            Objects.ramseteController, 
            Constants.kTankKinematics, 
            Objects.feedforward, 
            this::getWheelSpeeds, 
            new PIDConstants(Constants.kPDrive, Constants.kIDrive, Constants.kDDrive), 
            this::setTankDriveVolts, 
            eventMap, 
            useColor, 
            drive);

        return builder.fullAuto(traj);
    }

    public Pose2d getPose() {
        return Objects.poseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d pose) {
        Objects.leftFrontEncoder.setPosition(0);
        Objects.leftBackEncoder.setPosition(0);
        Objects.rightFrontEncoder.setPosition(0);
        Objects.rightBackEncoder.setPosition(0);

        Objects.poseEstimator.resetPosition(getHeading(), getLeftFrontPosition(), getRightFrontPosition(), pose);
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftFrontVelocity(), getRightFrontVelocity());
    }

    public void setTankDriveVolts(double rightVolts, double leftVolts) {
        Objects.rightFront.setVoltage(rightVolts);
        Objects.leftFront.setVoltage(leftVolts);
        Objects.robotDrive.feed();
    }

    /////// Odometry Periodic \\\\\\\\
    public void updateOdometry() {
        Objects.poseEstimator.update(getHeading(), getLeftFrontPosition(), getRightFrontPosition());
    }

    public void addVision(Pose2d pose, double timestamp) {
        Objects.poseEstimator.addVisionMeasurement(pose, timestamp, Constants.kVisionMeasurementStdDevs);
    }


    ////// GETTERS AND SETTERS \\\\\\\\
    // Gyro
    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(Objects.navX.getYaw());
    }

    public void resetGyro() {
        Objects.navX.setAngleAdjustment(getHeading().getDegrees());;
    }

    // Encoders
    public double getLeftFrontVelocity() {
        return getVelocity(Objects.leftFrontEncoder, Constants.kLeftInvert);
    }

    public double getLeftBackVelocity() {
        return getVelocity(Objects.leftBackEncoder, Constants.kLeftInvert);
    }

    public double getRightFrontVelocity() {
        return getVelocity(Objects.rightFrontEncoder, Constants.kRightInvert);
    }

    public double getRightBackVelocity() {
        return getVelocity(Objects.rightBackEncoder, Constants.kRightInvert);
    }

    public double getLeftFrontPosition() {
        return getPosition(Objects.leftFrontEncoder, Constants.kLeftInvert);
    }

    public double getLeftBackPosition() {
        return getPosition(Objects.leftBackEncoder, Constants.kLeftInvert);
    }

    public double getRightFrontPosition() {
        return getPosition(Objects.rightFrontEncoder, Constants.kRightInvert);
    }

    public double getRightBackPosition() {
        return getPosition(Objects.rightBackEncoder, Constants.kRightInvert);
    }

    public double getVelocity(RelativeEncoder encoder, boolean inverted) {
        return inverted ? -encoder.getVelocity() : encoder.getVelocity();
    }

    public double getPosition(RelativeEncoder encoder, boolean inverted) {
        return inverted ? -encoder.getPosition() : encoder.getPosition();
    }
}