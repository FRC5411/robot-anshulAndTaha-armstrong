// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.systems.arm.ArmSubsystem;
import frc.robot.systems.drive.DriveSubsystem;
import frc.robot.systems.intake.IntakeSubsystem;

public class AutonManager extends SubsystemBase {

  private DriveSubsystem robotDrive;
  private ArmSubsystem robotArm;
  private IntakeSubsystem robotIntake;

  /** Creates a new AutonManager. */
  public AutonManager(DriveSubsystem robotDrive, ArmSubsystem robotArm, IntakeSubsystem robotIntake) {
    this.robotDrive = robotDrive;
    this.robotArm = robotArm;
    this.robotIntake = robotIntake;
  }

  /*
   * Scores cone high, idles arm, goes for mobility
   */
  public Command coneMobility() {
    HashMap<String, Command> map = new HashMap<>();

    map.put("ScoreConeHigh", robotArm.armPIDAuton(robotArm, "high", true));
    map.put("OuttakeCone", robotIntake.intakeCommand(false, true));

    map.put("IdleArm", robotArm.armPIDAuton(robotArm, "idle", false));

    return robotDrive.getAutonomousCommand("ConeHighMobility", true, map, robotDrive);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
