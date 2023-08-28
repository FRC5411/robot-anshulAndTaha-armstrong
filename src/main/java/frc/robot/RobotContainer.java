// In Java We Trust

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.systems.arm.ArmSubsystem;
import frc.robot.systems.drive.DriveSubsystem;
import frc.robot.systems.intake.IntakeSubsystem;
import frc.robot.ControllerVars.Objects;
import frc.robot.auton.AutonManager;

public class RobotContainer {
  private DriveSubsystem robotDrive; 
  private ArmSubsystem robotArm; 
  private IntakeSubsystem robotIntake;

  private AutonManager autonManager;

  private SendableChooser<Command> driverChooser;

  public RobotContainer() {
    robotDrive = new DriveSubsystem();
    robotArm = new ArmSubsystem();
    robotIntake = new IntakeSubsystem();

    autonManager = new AutonManager(robotDrive, robotArm, robotIntake);

    driverChooser = new SendableChooser<>();
    Shuffleboard.getTab("Driver Profiles").add(driverChooser);

    robotDrive.setDefaultCommand(
      robotDrive.arcadeCmd(
        () -> Objects.xboxController.getLeftY(), 
        () -> Objects.xboxController.getRightX(), 
        () -> RobotStates.sDriveSniperMode));

    configureBindings();
    configureDriveProfiles();
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
      .whileFalse(shouldHoldArm());
    Objects.armMid
      .whileTrue(robotArm.armPIDTeleop(robotArm, "mid"))
      .whileFalse(shouldHoldArm());
    Objects.armLow
      .whileTrue(robotArm.armPIDTeleop(robotArm, "low"))
      .whileFalse(shouldHoldArm());

    Objects.toggleMode
      .toggleOnTrue(new InstantCommand(() -> {RobotStates.sIsConeMode = true;}))
      .toggleOnFalse(new InstantCommand(() -> {RobotStates.sIsConeMode = false;}));

    Objects.armGround
      .whileTrue(robotArm.armPIDTeleop(robotArm, "ground"))
      .whileFalse(shouldHoldArm());
    Objects.armSub
      .whileTrue(robotArm.armPIDTeleop(robotArm, "substation"))
      .whileFalse(shouldHoldArm());

    Objects.armIdle
      .whileTrue(robotArm.armPIDTeleop(robotArm, "idle"))
      .whileFalse(shouldHoldArm());

    Objects.holdSniper
      .whileTrue(new InstantCommand(() -> {RobotStates.sArmSniperMode = true;}))
      .whileFalse(new InstantCommand(() -> {RobotStates.sArmSniperMode = false;}));

    Objects.armOut
      .whileTrue(new InstantCommand(() -> {robotArm.setArmTeleop(0.3); RobotStates.sShouldHoldArm = false;}))
      .whileFalse(shouldHoldArm());
    Objects.armIn
      .whileTrue(new InstantCommand(() -> {robotArm.setArmTeleop(-0.3); RobotStates.sShouldHoldArm = false;}))
      .whileFalse(shouldHoldArm());

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

  private void configureDriveProfiles() {
    driverChooser.addOption("Aaron", new InstantCommand(() -> {
      RobotStates.sDeadzones = 0.1;
      RobotStates.sShouldSquareInputs = true;
    }));
    
    driverChooser.addOption("System Defaults", new InstantCommand(() -> {
      RobotStates.sDeadzones = 0.0;
      RobotStates.sShouldSquareInputs = false;
    }));
  }

  private Command shouldHoldArm() {
    return new InstantCommand(() -> {
      RobotStates.sShouldHoldArm = true;

      robotArm.setArmTeleop(0.0);
      robotArm.armFFHold(robotArm).schedule();
    });
  }

  public Command getAutonomousCommand() {
    return autonManager.coneMobility();
  }
}