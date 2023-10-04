// In Java We Trust

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.systems.arm.ArmSubsystem;
import frc.robot.systems.arm.ArmVars;
import frc.robot.systems.drive.DriveSubsystem;
import frc.robot.systems.intake.IntakeSubsystem;
import frc.robot.systems.intake.IntakeVars;
import frc.robot.structure.operator.ButtonMap.Map;
import frc.robot.structure.operator.ButtonMap.Objects;
import frc.robot.structure.operator.OperatorProfiles.Aaron;
import frc.robot.structure.operator.OperatorProfiles.Profile;
import frc.robot.structure.operator.OperatorProfiles.SystemDefault;
import frc.robot.structure.robot.SystemVerification;

public class RobotContainer {

  private static SystemVerification m_robotVerification = new SystemVerification();

  private static DriveSubsystem m_robotDrive = m_robotVerification.verifyRobotDrive();
  private static ArmSubsystem m_robotArm = m_robotVerification.verifyRobotArm();
  private static IntakeSubsystem m_robotIntake = m_robotVerification.verifyRobotIntake();

  private static PowerDistribution m_robotPDH = new PowerDistribution(0, ModuleType.kRev);

  private static LoggedDashboardChooser<Command> m_driverChooser = new LoggedDashboardChooser<>("DriverChooser");

  public Profile m_driverProfile = new SystemDefault();

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

    /* Set the default command for the arm subsystem to hold it's last position */
    // m_robotArm.setDefaultCommand(
    //     // NOTE: Setpoint does nothing since isHolding is true
    //     m_robotArm.MoveArmCommand("HOLD", true));

    /* Set the default command for the intake subsystem to not run the motor */
    m_robotIntake.setDefaultCommand(
      Commands.runOnce(() -> m_robotIntake.setSpeeds(0.0), m_robotIntake));

    Shuffleboard.getTab("Driver").add(m_driverChooser.getSendableChooser());
    m_driverChooser.addDefaultOption("SystemDefaults", setProfile(new SystemDefault()));
    m_driverChooser.addOption("Aaron", setProfile(new Aaron()));

    configureBindings();
  }

  private void configureBindings() {
    m_driverProfile.configureBindings(m_robotDrive);

    // NOTE: Test function for sim
    // Objects.DRIVER_CONTROLLER.a().whileTrue(new InstantCommand(() -> {
    //   m_robotArm.setSpeeds(6.0);
    // })).whileFalse(new InstantCommand(() -> {
    //   m_robotArm.setSpeeds(0.0);
    // }));

    // Objects.DRIVER_CONTROLLER.b().whileTrue(m_robotArm.MoveArmCommand("HIGH", false))
    //     .whileFalse(new InstantCommand(() -> {
    //     }));
    buttonArmToSetpoint(
      Objects.OPERATOR_CONTROLLER.button(Map.B_ARM_HIGH), "HIGH");

    buttonArmToSetpoint(
      Objects.OPERATOR_CONTROLLER.button(Map.B_ARM_MID), "MID");

    buttonArmToSetpoint(
      Objects.OPERATOR_CONTROLLER.button(Map.B_ARM_LOW), "LOW");

    buttonArmToSetpoint(
      Objects.OPERATOR_CONTROLLER.button(Map.B_ARM_SUB), "SUB");

    buttonArmToSetpoint(
      Objects.OPERATOR_CONTROLLER.button(Map.B_ARM_GROUND), "GRND");

    buttonArmToSetpoint(
      Objects.OPERATOR_CONTROLLER.button(Map.B_ARM_IDLE), "IDLE");

    buttonManualArm(
      Objects.OPERATOR_CONTROLLER.button(Map.B_ARM_IN_MANUAL), 0.5);

    buttonManualArm(
      Objects.OPERATOR_CONTROLLER.button(Map.B_ARM_OUT_MANUAL), -0.5);

    buttonRunIntake(
      Objects.OPERATOR_CONTROLLER.button(Map.B_INTAKE_IN), true);

    buttonRunIntake(
      Objects.OPERATOR_CONTROLLER.button(Map.B_INTAKE_OUT), false);

    buttonToggleMode(Objects.OPERATOR_CONTROLLER.button(Map.B_TOGGLE_MODE));

    // buttonSniperMode(Objects.OPERATOR_CONTROLLER.button(ControllerConstants.kSniper));
  }

  /* ----------------------------------------------------------------------------- */
  /* Custom button board methods */
  private void buttonArmToSetpoint(Trigger button, String position) {
    button.whileTrue(
      m_robotArm.MoveArmCommand(position, false)
    ).whileFalse(
      new InstantCommand()
    );
  }

  private void buttonManualArm(Trigger button, double speed) {
    button.whileTrue(
      Commands.runOnce( () -> m_robotArm.setSpeeds(speed), m_robotArm)
    ).whileFalse(
      new InstantCommand()
    );
  }

  private void buttonRunIntake(Trigger button, boolean isIntaking) {
    button.whileTrue(
      Commands.runOnce(isIntaking ? () -> m_robotIntake.intake(null) : () -> m_robotIntake.outtake(null), 
        m_robotIntake)
    ).whileFalse(
      new InstantCommand()
    );
  }

  private void buttonToggleMode(Trigger button) {
    button.toggleOnTrue(
      Commands.runOnce( () -> ArmVars.Vars.IS_CONE = true, m_robotArm )
      .andThen(Commands.runOnce( () -> IntakeVars.Vars.IS_CONE = true ))
      .andThen(Commands.runOnce( () -> m_robotPDH.setSwitchableChannel(true) ))
    ).toggleOnFalse(
      Commands.runOnce( () -> ArmVars.Vars.IS_CONE = false, m_robotArm )
      .andThen(Commands.runOnce( () -> IntakeVars.Vars.IS_CONE = false ))
      .andThen(Commands.runOnce( () -> m_robotPDH.setSwitchableChannel(false) ))
    );
  }

  // private void buttonSniperMode(Trigger button) {
  //   button.whileTrue(
  //     Commands.runOnce( () -> DriveVars.Vars., m_robotDrive)
  //   ).whileFalse(
  //     Commands.runOnce( () -> m_robotDrive.setSniperCommand(false), m_robotDrive)
  //   );
  // }

  /* ----------------------------------------------------------------------------- */

  /**
   * Command to initialize profiles when teleop is enabled
   * 
   * @return Command to set the selected profile
   */
  public Command initProfiles() {
    return m_driverChooser.get();
  }

  /**
   * Command to set the profile and update the button bindings
   * 
   * @param profile Selected driver profile
   * @return InstantCommand() that sets the profile instance var and calls
   *         configureBindings()
   */
  private Command setProfile(Profile profile) {
    return new InstantCommand(() -> {
      m_driverProfile = profile;
      configureBindings();
    });
  }

  public Command getAutonomousCommand() {
    return new PrintCommand("No autonomous configured");
  }
}

// return new FunctionalCommand(
// /* Init Code */
// () -> {},
// /* Execute Code */
// () -> {},
// /* End code */
// interrupted -> {},
// /* Is finished */
// () -> false,
// /* Required subsystems */
// this
// );
// }