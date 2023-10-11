// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.drive;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.systems.drive.DriveVars.Constants;
import frc.robot.systems.drive.DriveVars.Objects;
import frc.robot.systems.drive.DriveVars.Simulation;
import frc.robot.systems.drive.DriveVars.Vars;
import java.util.HashMap;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/** Drivetrain subsytem class */
public class DriveSubsystem extends SubsystemBase {

  private final DriveIO m_driveIO;
  private final GyroIO m_gyroIO;

  private final DriveIOInputsAutoLogged m_driveInputs = new DriveIOInputsAutoLogged();
  private final GyroIOInputsAutoLogged m_gyroInputs = new GyroIOInputsAutoLogged();

  /**
   * Creates a new instance of the drive subsystem.
   *
   * @param driveIO Type of IO the subsystem will be using
   * @param gyroIO Type of IO the gyro will be using
   */
  public DriveSubsystem(DriveIO driveIO, GyroIO gyroIO) {
    m_driveIO = driveIO;
    m_gyroIO = gyroIO;

    // Need to calibrate and reset the heading
    Objects.NAVX.calibrate();
    resetGyroYaw();
  }

  @Override
  public void periodic() {
    /* Update and process drive motor inputs */
    m_driveIO.updateInputs(m_driveInputs);
    Logger.getInstance().processInputs("/systems/drive/driveIO", m_driveInputs);

    /* Update and process gyro inputs (Technically outputs but oh well) */
    m_gyroIO.updateInputs(m_gyroInputs);
    Logger.getInstance().processInputs("/systems/drive/gyroIO", m_gyroInputs);

    /* Update pose estimator (odometry) */
    Objects.ROBOT_POSE_ESTIMATOR.update(
        m_gyroInputs.gyroYawDeg, m_driveInputs.leftFrontPosition, m_driveInputs.rightFrontPosition);
    Logger.getInstance()
        .recordOutput(
            "/systems/drive/estimatedPose", Objects.ROBOT_POSE_ESTIMATOR.getEstimatedPosition());

    /* Update Field2d pose */
    Objects.ROBOT_FIELD.setRobotPose(Objects.ROBOT_POSE_ESTIMATOR.getEstimatedPosition());
    SmartDashboard.putData(Objects.ROBOT_FIELD);
  }

  @Override
  public void simulationPeriodic() {
    /* Update and process drive motor inputs */
    m_driveIO.updateInputs(m_driveInputs);
    Logger.getInstance().processInputs("/systems/drive/driveIO", m_driveInputs);

    /* Updates simulation's time */
    Simulation.lasttime += 0.02;

    /* Updates the simulation plant */
    Simulation.DRIVE_SIMULATOR.update(0.02);

    /* Updates the simulation gyro */
    Simulation.GYRO_SIMULATOR.setAngle(m_gyroInputs.gyroYawDeg.getDegrees());
  }

  /** Resets the yaw of the gyro */
  public void resetGyroYaw() {
    Objects.NAVX.reset();
  }

  /**
   * Sets the wheel speeds using voltage
   *
   * @param leftVolts voltage for the left side
   * @param rightVolts voltage for the right side
   */
  public void setSpeeds(double leftVolts, double rightVolts) {
    m_driveIO.setVolts(leftVolts * 12.0, rightVolts * 12.0);
    Objects.ROBOT_DRIVE.feed();
  }

  /**
   * Simple arcade drive method to drive the robot
   *
   * @param yInput Forward speed of the robot (Typically left joystick Y)
   * @param xInput Rotational speed of the robot (Typically right joystick X)
   * @param squareInputs Should square inputs (true or false)
   */
  public void arcadeDrive(final double yInput, final double xInput, final boolean squareInputs) {
    var wheelSpeeds = DifferentialDrive.arcadeDriveIK(yInput, xInput, squareInputs);

    setSpeeds(
        wheelSpeeds.left * DifferentialDrive.kDefaultMaxOutput,
        wheelSpeeds.right * DifferentialDrive.kDefaultMaxOutput);
  }

  /**
   * An inline command for an Arcade Drive
   *
   * @param yInputSupplier Forward, backward speed
   * @param xInputSupplier Rotational speed
   * @param sniperMode Should make robot slower
   * @param isTeleop Make adjustments to inputs or not
   * @return A new arcade command
   */
  public Command ArcadeCommand(
      DoubleSupplier yInputSupplier,
      DoubleSupplier xInputSupplier,
      BooleanSupplier sniperMode,
      boolean isTeleop) {
    return new FunctionalCommand(
        /* Init Code */
        () -> {},
        /* Execute Code */
        () -> {
          /* Initialise all necessary vars */
          double ySpeed = yInputSupplier.getAsDouble();
          double xSpeed = xInputSupplier.getAsDouble();

          final double kDeadzone = Vars.DEADZONE;
          final boolean kSquareInputs = Vars.SQUARE_INPUTS;
          // NOTE: You must use a boolean supplier for sniper mode
          // since in teleop you may want to toggle it or not
          boolean isSniper = sniperMode.getAsBoolean();

          /* If we are using this command in teleop or auton */
          if (isTeleop) {
            /* Check deadzones */
            if (Math.abs(ySpeed) < kDeadzone) {
              ySpeed = 0.0;
            }
            if (Math.abs(xSpeed) < kDeadzone) {
              xSpeed = 0.0;
            }

            /* Check if drive should be in sniper mode */
            if (isSniper) {
              ySpeed *= Constants.SPEED_SCALER;
              xSpeed *= Constants.SPEED_SCALER;
            }

            arcadeDrive(ySpeed, xSpeed, kSquareInputs);
          } else {
            arcadeDrive(ySpeed, xSpeed, false);
          }
        },
        /* End Code */
        // NOTE: interrupted is a boolean
        interrupted -> {},
        /* Is finished */
        () -> false,
        /* Required subsystems */
        this);
  }

  /**
   * An inline command to balance on the charge station
   *
   * @return A new balance command
   */
  public Command BalanceCommand() {
    // TODO: Adjust FF to be an arm FF
    final ProfiledPIDController kController =
        new ProfiledPIDController(0.0243, 0.0, 0.0, new TrapezoidProfile.Constraints(1.0, 0.5));
    final double kF = 0.05;

    return new FunctionalCommand(
        /* Init Code */
        () -> {},
        /* Execute Code */
        () -> {
          // NOTE: xInput and squareInputs should be left to 0.0 and false respectively
          arcadeDrive(
              /* Calculates the drive output and accounts for static friction */
              kController.calculate(m_gyroInputs.gyroPitchDeg.getDegrees(), 0.0)
                  + kF * Math.signum(kController.getPositionError()),
              0.0,
              false);
        },
        /* End Code */
        // NOTE: interrupted is a boolean
        interrupted -> {
          /* Set motor speeds to 0 */
          arcadeDrive(0.0, 0.0, false);
        },
        /* Is finished */
        () -> false,
        /* Required subsystems */
        this);
  }

  /**
   * Command that will follow a trajectory and it's plotted events
   *
   * @param trajectory Trajectory to follow (filename)
   * @param useAlliance Should change path based on your current alliance
   * @param eventMap Map of events
   * @return Autonomous command that will follow a path and execute events
   */
  public Command followPathWithEventsCommand(
      String trajectory, boolean useAlliance, HashMap<String, Command> eventMap) {
    /* Init path vars */
    PathConstraints pathConstraints = PathPlanner.getConstraintsFromPath(trajectory);
    List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup(trajectory, pathConstraints);
    PathPlannerTrajectory mapPath = PathPlanner.loadPath(trajectory, pathConstraints);

    /* Display the trajectory on the field obj */
    Objects.ROBOT_FIELD.getObject(trajectory).setTrajectory(mapPath);

    /* Reset encoder poses */
    Objects.LEFT_FRONT_ENCODER.setPosition(0.0);
    Objects.LEFT_BACK_ENCODER.setPosition(0.0);
    Objects.RIGHT_FRONT_ENCODER.setPosition(0.0);
    Objects.RIGHT_BACK_ENCODER.setPosition(0.0);

    /* Build path with events */
    // TODO: Fix reset pose consumer
    RamseteAutoBuilder pathBuilder =
        new RamseteAutoBuilder(
            () -> Objects.ROBOT_POSE_ESTIMATOR.getEstimatedPosition(),
            pose ->
                Objects.ROBOT_POSE_ESTIMATOR.resetPosition(
                    m_gyroInputs.gyroYawDeg,
                    m_driveInputs.leftFrontPosition,
                    m_driveInputs.rightFrontPosition,
                    pose),
            Objects.DRIVE_RAMSETE_CONTROLLER,
            Constants.DRIVE_KINEMATICS,
            Objects.DRIVE_FEEDFORWARD,
            () ->
                new DifferentialDriveWheelSpeeds(
                    m_driveInputs.leftFrontVelocity, m_driveInputs.rightFrontVelocity),
            new PIDConstants(Constants.DRIVE_PP_P, Constants.DRIVE_PP_I, Constants.DRIVE_PP_D),
            this::setSpeeds,
            eventMap,
            useAlliance,
            this);

    return pathBuilder.fullAuto(path);
  }
}
