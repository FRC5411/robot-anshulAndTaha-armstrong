// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.intake;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotStates;
import frc.robot.systems.intake.IntakeVars.Constants;

public class IntakeSubsystem extends SubsystemBase {

  private IntakeIO intakeIO;

  private boolean safeToSpin;

  public IntakeSubsystem() {
    intakeIO = new IntakeIO();
    safeToSpin = false;
  }

  public void setIntakeSpin(double speed) {
    if (safeToSpin) intakeIO.setIntake(speed);
  }

  public void intakeIn() {
    if (RobotStates.sIsConeMode) intakeIO.setIntake(Constants.kIntakeConeSpeed);
    else intakeIO.setIntake(-Constants.kIntakeCubeSpeed);
  }

  public void intakeOut() {
    if (RobotStates.sIsConeMode) intakeIO.setIntake(-Constants.kIntakeConeSpeed);
    else intakeIO.setIntake(Constants.kIntakeCubeSpeed);
  }

  public void intakeOff() {
    intakeIO.setIntake(0.0);
  }

  // TO-DO: This method still needs to be tested to ensure it works the same as on the old code
  public void safeSpinIntake() {
    LinearFilter filter = LinearFilter.movingAverage(25);
    double calc = filter.calculate(intakeIO.getIntakeCurrent());

    if (calc > Constants.kIntakeCurrentLimit) { safeToSpin = false; }
    safeToSpin = true;
}

  @Override
  public void periodic() {
    safeSpinIntake();
  }
}
