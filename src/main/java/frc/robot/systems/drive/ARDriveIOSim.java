package frc.robot.systems.drive;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
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
public class ARDriveIOSim {
    public ARDriveIOSim() {
        SmartDashboard.putData(Simulation.field);
    }

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
    public Command followPathwithEvents(String trajPath, boolean useColor, HashMap<String, Command> eventMap, Subsystem drive) {
        PathConstraints pathConstraints = PathPlanner.getConstraintsFromPath(trajPath);
        List<PathPlannerTrajectory> mainTrajectory = PathPlanner.loadPathGroup(trajPath, pathConstraints);
        PathPlannerTrajectory mapTrajectory = PathPlanner.loadPath(trajPath, pathConstraints);
        Simulation.field.getObject(trajPath).setTrajectory(mapTrajectory);

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
        Simulation.lasttime += 0.02;
        Simulation.driveSim.setInputs(Objects.leftFront.get() * 12, Objects.rightFront.get() * 12);
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
        SmartDashboard.putNumber("SimDrive/Chassis/X", getPose().getX());
        SmartDashboard.putNumber("SimDrive/Chassis/Y", getPose().getY());
        SmartDashboard.putNumber("SimDrive/Chassis/Degrees", getPose().getRotation().getDegrees());

        SmartDashboard.putNumber("SimDrive/Chassis/XMPS", getChassisSpeeds().vxMetersPerSecond);
        SmartDashboard.putNumber("SimDrive/Chassis/YMPS", getChassisSpeeds().vyMetersPerSecond);
        SmartDashboard.putNumber("SimDrive/Chassis/DegreesMPS", Math.toRadians(getChassisSpeeds().omegaRadiansPerSecond));

        SmartDashboard.putNumber("SimDrive/Chassis/LeftVelocity", getLeftVelocity());
        SmartDashboard.putNumber("SimDrive/Chassis/RightVelocity", getRightVelocity());

        SmartDashboard.putNumber("SimDrive/Chassis/LeftPosition", getLeftPosition());
        SmartDashboard.putNumber("SimDrive/Chassis/RightPosition", getRightPosition());

        SmartDashboard.putNumber("SimDrive/Chassis/LeftVolts", Objects.leftFront.get() * 12);
        SmartDashboard.putNumber("SimDrive/Chassis/RightVolts", Objects.rightFront.get() * 12);

        SmartDashboard.putNumber("SimDrive/Chassis/LastTime", Simulation.lasttime);
    }

    public void setField() {
        Simulation.field.setRobotPose(getPose());
    }
}