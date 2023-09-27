// In Java We Trust

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.systems.drive.DriveSubsystem;
import frc.robot.structure.operator.ButtonMap.Objects;
import frc.robot.structure.operator.OperatorProfiles.Aaron;
import frc.robot.structure.operator.OperatorProfiles.Profile;
import frc.robot.structure.operator.OperatorProfiles.SystemDefault;
import frc.robot.structure.robot.SystemVerification;

public class RobotContainer {

  private static SystemVerification m_robotVerification = new SystemVerification();

  private static DriveSubsystem m_robotDrive = m_robotVerification.verifyRobotDrive();

  private static SendableChooser<Runnable> m_driverChooser = new SendableChooser<>();

  public Profile m_driverProfile;

  public RobotContainer() {
    /*
     * Sets the drive subsystem's default command (Command that runs when no other
     * commands require drive)
     */
    m_robotDrive.setDefaultCommand(
        // NOTE: Joysticks are inverted on XboxController by default, so we need to
        // un-invert them
        m_robotDrive.ArcadeCommand(
            () -> -Objects.DRIVER_CONTROLLER.getLeftY(),
            () -> -Objects.DRIVER_CONTROLLER.getRightX(),
            () -> Objects.DRIVER_CONTROLLER.leftTrigger().getAsBoolean(),
            true));

    Shuffleboard.getTab("Driver").add(m_driverChooser);
    m_driverChooser.setDefaultOption("SystemDefaults", () -> { m_driverProfile = new SystemDefault(); });
    m_driverChooser.addOption("Aaron", () -> { m_driverProfile = new Aaron(); });

    configureBindings();
  }

  private void configureBindings() {
    m_driverProfile.configureBindings(m_robotDrive);
  }

  public Command getAutonomousCommand() {
    return new PrintCommand("No autonomous configured");
  }
}