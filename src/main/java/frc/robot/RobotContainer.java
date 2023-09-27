// In Java We Trust

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.systems.drive.DriveSubsystem;
import frc.robot.ControllerVars.Objects;
import frc.robot.structure.robot.SystemVerification;

public class RobotContainer {

  private static SystemVerification m_robotVerification = new SystemVerification();

  private static DriveSubsystem m_robotDrive = m_robotVerification.verifyRobotDrive();

  public RobotContainer() {
    /*
     * Sets the drive subsystem's default command (Command that runs when no other
     * commands require drive)
     */
    m_robotDrive.setDefaultCommand(
        // NOTE: Joysticks are inverted on XboxController by default, so we need to
        // un-invert them
        m_robotDrive.ArcadeCommand(
            () -> -Objects.xboxController.getLeftY(),
            () -> -Objects.xboxController.getLeftX(),
            () -> Objects.xboxController.leftTrigger().getAsBoolean(),
            true));

    configureBindings();
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return new PrintCommand("No autonomous configured");
  }
}