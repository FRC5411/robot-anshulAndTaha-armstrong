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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import frc.robot.systems.drive.DriveVars.Constants;
import frc.robot.systems.drive.DriveVars.Objects;

import com.revrobotics.CANSparkMax;

import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathConstraints;

import frc.robot.RobotStates;

/** Add your docs here. */
public class DriveIO {
    public DriveIO() {
        SmartDashboard.putData(Objects.field);
    }

    ////// TELEOP \\\\\\\
    public void arcadeDrive(double speed, double rotation) {
        Objects.robotDrive.arcadeDrive(speed, rotation);
    }

    public void teleopDrive(double speed, double rotation, boolean snipermode) {
        if (Math.abs(speed) < Constants.kDeadzone) {
            speed = 0;
        }

        if (Math.abs(rotation) < Constants.kDeadzone) {
            rotation = 0;
        }

        rotation *= Constants.kRotationScaler;

        if (snipermode) {
            speed *= Constants.kSniperScaler;
            rotation *= Constants.kSniperScaler;
        }

        Objects.robotDrive.arcadeDrive(speed, rotation);
    }

    /////// PATH PLANNER \\\\\\\
    public Command followPathwithEvents(String trajPath, boolean useColor, HashMap<String, Command> eventMap, Subsystem drive) {
        PathConstraints pathConstraints = PathPlanner.getConstraintsFromPath(trajPath);
        List<PathPlannerTrajectory> mainTrajectory = PathPlanner.loadPathGroup(trajPath, pathConstraints);
        PathPlannerTrajectory mapTrajectory = PathPlanner.loadPath(trajPath, pathConstraints);
        Objects.field.getObject(trajPath).setTrajectory(mapTrajectory);

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

        return builder.fullAuto(mainTrajectory);
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

    public ChassisSpeeds getChassisSpeeds() {
        return Constants.kTankKinematics.toChassisSpeeds(getWheelSpeeds());
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

    public double getPitch() {
        return Objects.navX.getPitch();
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

    // Telemetry
    public void telemetry() {
        telemetryLF();
        telemetryLB();
        telemetryRF();
        telemetryRB();

        SmartDashboard.putNumber("Drive/Chassis/X", getPose().getX());
        SmartDashboard.putNumber("Drive/Chassis/Y", getPose().getY());
        SmartDashboard.putNumber("Drive/Chassis/Degrees", getPose().getRotation().getDegrees());

        SmartDashboard.putNumber("Drive/Speed/XMPS", getChassisSpeeds().vxMetersPerSecond);
        SmartDashboard.putNumber("Drive/Speed/YMPS", getChassisSpeeds().vyMetersPerSecond);
        SmartDashboard.putNumber("Drive/Speed/DegreesMPS", Math.toRadians(getChassisSpeeds().omegaRadiansPerSecond));

        SmartDashboard.putBoolean("Drive/SniperMode", RobotStates.sDriveSniperMode);
    }

    public void telemetryLF() {
        telemetryMotor("Drive/LF/", Objects.leftFront, Objects.leftFrontEncoder);
    }

    public void telemetryLB() {
        telemetryMotor("Drive/LB/", Objects.leftBack, Objects.leftBackEncoder);
    }

    public void telemetryRF() {
        telemetryMotor("Drive/RF/", Objects.rightFront, Objects.rightBackEncoder);
    }

    public void telemetryRB() {
        telemetryMotor("Drive/RB/", Objects.rightBack, Objects.rightBackEncoder);
    }

    public void telemetryMotor(String key, CANSparkMax motor, RelativeEncoder encoder) {
        SmartDashboard.putNumber(key + "Position", encoder.getPosition());
        SmartDashboard.putNumber(key + "Velocity", encoder.getVelocity());
        SmartDashboard.putNumber(key + "Voltage", motor.get() * 12);
        SmartDashboard.putNumber(key + "BusVoltage", motor.getBusVoltage());
        SmartDashboard.putNumber(key + "Temperature", motor.getMotorTemperature());
        SmartDashboard.putNumber(key + "DutyCycle", motor.getAppliedOutput());
        SmartDashboard.putNumber(key + "Amps", motor.getOutputCurrent());
        SmartDashboard.putNumber(key + "ID", motor.getDeviceId());
    }

    public void setField() {
        Objects.field.setRobotPose(getPose());
    }

    public void trueSniperMode() {
        RobotStates.sDriveSniperMode = true;
    }

    public void falseSniperMode() {
        RobotStates.sDriveSniperMode = false;
    }
}