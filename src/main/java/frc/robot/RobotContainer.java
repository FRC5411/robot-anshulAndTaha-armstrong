// In Java We Trust

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
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
    // Objects.a.onTrue(robotDrive.autoEngageCmd()); // Auto Engage no work DO NOT RUN THIS
    // Objects.b.onTrue(robotDrive.turnCommand(180));
    Objects.y
      .onTrue(robotDrive.resetOdometryCmd());

    Objects.leftTrigger
      .whileTrue(robotDrive.sniperTrueCmd());
    Objects.rightTrigger
      .whileTrue(robotDrive.sniperFalseCmd());

    // Arm Bindings
    Objects.armHigh
      .whileTrue(robotArm.armPIDTeleop(robotArm, "high"))
      .whileFalse(new InstantCommand(() -> {robotArm.setArmTeleop(0.0);}));
    Objects.armMid
      .whileTrue(robotArm.armPIDTeleop(robotArm, "mid"))
      .whileFalse(new InstantCommand(() -> {robotArm.setArmTeleop(0.0);}));
    Objects.armLow
      .whileTrue(robotArm.armPIDTeleop(robotArm, "low"))
      .whileFalse(new InstantCommand(() -> {robotArm.setArmTeleop(0.0);}));

    Objects.toggleMode
      .toggleOnTrue(new InstantCommand(() -> {RobotStates.sIsConeMode = true;}))
      .toggleOnFalse(new InstantCommand(() -> {RobotStates.sIsConeMode = false;}));

    Objects.armGround
      .whileTrue(robotArm.armPIDTeleop(robotArm, "ground"))
      .whileFalse(new InstantCommand(() -> {robotArm.setArmTeleop(0.0);}));
    Objects.armSub
      .whileTrue(robotArm.armPIDTeleop(robotArm, "substation"))
      .whileFalse(new InstantCommand(() -> {robotArm.setArmTeleop(0.0);}));

    Objects.armIdle
      .whileTrue(robotArm.armPIDTeleop(robotArm, "idle"))
      .whileFalse(new InstantCommand(() -> {robotArm.setArmTeleop(0.0);}));

    Objects.holdSniper
      .whileTrue(new InstantCommand(() -> {RobotStates.sDriveSniperMode = true;}))
      .whileFalse(new InstantCommand(() -> {RobotStates.sDriveSniperMode = false;}));

    Objects.armOut
      .whileTrue(new InstantCommand(() -> {robotArm.setArmTeleop(0.3); shouldHoldArm(false);}))
      .whileFalse(new InstantCommand(() -> {robotArm.setArmTeleop(0.0); shouldHoldArm(true);}));
    Objects.armIn
      .whileTrue(new InstantCommand(() -> {robotArm.setArmTeleop(-0.3); shouldHoldArm(false);}))
      .whileFalse(new InstantCommand(() -> {robotArm.setArmTeleop(0.0); shouldHoldArm(true);}));

    // Intake Bindings
    Objects.leftBumper
      .whileTrue(new InstantCommand(robotIntake :: intakeIn))
      .whileFalse(new InstantCommand(robotIntake :: intakeOff));
    Objects.rightBumper
      .whileTrue(new InstantCommand(robotIntake :: intakeOut))
      .whileFalse(new InstantCommand(robotIntake :: intakeOff));
    Objects.intakeButton
      .whileTrue(new InstantCommand(robotIntake :: intakeIn))
      .whileFalse(new InstantCommand(robotIntake :: intakeOff));
    Objects.outtakeButton
      .whileTrue(new InstantCommand(robotIntake :: intakeOut))
      .whileFalse(new InstantCommand(robotIntake :: intakeOff));
  }

  private Command shouldHoldArm(boolean hold) {
    if (hold) {
      return robotArm.armFFHold(robotArm);
    }
    else {
      return new PrintCommand("\nRC : shouldHoldArm " + hold);
    }
  }

  public Command getAutonomousCommand() {
    return robotDrive.getAutonomousCommand();
  }
}