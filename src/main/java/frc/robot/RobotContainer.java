// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.systems.arm.ArmSubsystem;
import frc.robot.systems.drive.DriveSubsystem;
import frc.robot.systems.intake.IntakeSubsystem;
import frc.robot.ControllerVars.Objects;

public class RobotContainer {
  private DriveSubsystem robotDrive; 
  private ArmSubsystem robotArm; 
  private IntakeSubsystem robotIntake; 

  public RobotContainer() {
    robotDrive = new DriveSubsystem();
    robotArm = new ArmSubsystem();
    robotIntake = new IntakeSubsystem();

    robotDrive.setDefaultCommand(
      robotDrive.arcadeCmd(
        () -> Objects.xboxController.getLeftY(), 
        () -> Objects.xboxController.getRightX(), 
        () -> RobotStates.sDriveSniperMode));

    configureBindings();
  }

  private void configureBindings() {
    // Drive Bindings
    Objects.a.onTrue(robotDrive.autoEngageCmd());
    Objects.b.onTrue(robotDrive.turnCommand(180));
    Objects.y.onTrue(robotDrive.resetOdometryCmd());

    Objects.leftTrigger.whileTrue(robotDrive.sniperTrueCmd());
    Objects.rightTrigger.whileTrue(robotDrive.sniperFalseCmd());

    // Arm Bindings
    
  }

  public Command getAutonomousCommand() {
    return robotDrive.getAutonomousCommand();
  }
}