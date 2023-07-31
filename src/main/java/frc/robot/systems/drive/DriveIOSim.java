package frc.robot.systems.drive;

import java.util.HashMap;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import frc.robot.systems.drive.DriveVars.Constants;
import frc.robot.systems.drive.DriveVars.Objects;
import frc.robot.systems.drive.DriveVars.Simulation;

/** Add your docs here. */
public class DriveIOSim {
    public DriveIOSim() {}

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
        return Simulation.driveSim.getPose();
    }

    public void resetPose(Pose2d pose) {
        Simulation.driveSim.setPose(pose);;
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
    }

    public ChassisSpeeds getChassisSpeeds() {
        return Constants.kTankKinematics.toChassisSpeeds(getWheelSpeeds());
    }

    public void update() {
        Simulation.driveSim.setInputs(Objects.leftFront.get() * 12, Objects.leftFront.get() * 12);
        Simulation.driveSim.update(0.02);
    }

    public void setTankDriveVolts(double rightVolts, double leftVolts) {
        Objects.rightFront.setVoltage(rightVolts);
        Objects.leftFront.setVoltage(leftVolts);
        Objects.robotDrive.feed();
    }

    ////// GETTERS AND SETTERS \\\\\\\\
    // Gyro
    public Rotation2d getHeading() {
        return Simulation.driveSim.getHeading();
    }

    // Encoders
    public double getLeftVelocity() {
        return Simulation.driveSim.getLeftVelocityMetersPerSecond();
    }

    public double getRightVelocity() {
        return Simulation.driveSim.getRightVelocityMetersPerSecond();
    }

    public double getLeftPosition() {
        return Simulation.driveSim.getLeftPositionMeters();
    }

    public double getRightPosition() {
        return Simulation.driveSim.getRightPositionMeters();
    }

    // Telemetry
    public void telemetry() {
        SmartDashboard.putNumber("Drive/Chassis/X", getPose().getX());
        SmartDashboard.putNumber("Drive/Chassis/Y", getPose().getY());
        SmartDashboard.putNumber("Drive/Chassis/Degrees", getPose().getRotation().getDegrees());

        SmartDashboard.putNumber("Drive/Chassis/XMPS", getChassisSpeeds().vxMetersPerSecond);
        SmartDashboard.putNumber("Drive/Chassis/YMPS", getChassisSpeeds().vyMetersPerSecond);
        SmartDashboard.putNumber("Drive/Chassis/DegreesMPS", Math.toRadians(getChassisSpeeds().omegaRadiansPerSecond));
    }

    public void setField() {
        Simulation.field.setRobotPose(getPose());
    }
}