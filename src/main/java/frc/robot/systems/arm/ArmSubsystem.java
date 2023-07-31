// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotStates;
import frc.robot.systems.arm.ArmVars.Constants;

public class ArmSubsystem extends SubsystemBase {
  ArmIO armIO;

  public ArmSubsystem() {
    armIO = new ArmIO();
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////

  public FunctionalCommand armHold(ArmSubsystem robotArm) {
    double kg = 0.03;

    ArmFeedforward feedForward = new ArmFeedforward(0, kg, 0, 0);
    
    return new FunctionalCommand(
      () -> System.out.println("Command HOLD ARM COMMAND has started"), 
      () -> {
        // 
        if(RobotStates.sShouldHoldArm){
          double calc = feedForward.calculate(ArmIO.getXAxisArmAngle(), 0);
    
          armIO.setArm(calc);
        }}, 
      interrupted -> {},
      () -> false, 
      robotArm);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////

  public FunctionalCommand armTeleop(ArmSubsystem robotArm, String strSetpoint){
    double kP = 0.066;
    double kI = 0;
    double kD = 0;

    ProfiledPIDController pid = new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(Constants.kArmVelocity, Constants.kArmAcceleration));
    
    return new FunctionalCommand(() -> {

      // Initialize Code
        pid.setTolerance(2);
        pid.reset(ArmIO.getArmAngle());

        System.out.println("Command Teleop ARM ALIGN has started");
      }, () -> {

      // Execute Code
        double calc = pid.calculate(ArmIO.getArmAngle(), returnAngle(strSetpoint));
        armIO.setArm(calc);
      }, 

      interrupted -> {}, 

      () -> false, 
      
      robotArm);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////

  private double returnAngle(String pos){
    switch(pos){
      case "high":
        return (RobotStates.sIsConeMode) ? Constants.ConeAngles.kHigh : Constants.CubeAngles.kHigh;

      case "mid":
        return (RobotStates.sIsConeMode) ? Constants.ConeAngles.kMid : Constants.CubeAngles.kMid;
        
      case "low":
        return (RobotStates.sIsConeMode) ? Constants.ConeAngles.kLow : Constants.CubeAngles.kLow;
        
      case "ground":
        return (RobotStates.sIsConeMode) ? Constants.ConeAngles.kGround : Constants.CubeAngles.kGround;
        
      case "substation":
        return (RobotStates.sIsConeMode) ? Constants.ConeAngles.kSubstation : Constants.CubeAngles.kSubstation;

      case "idle":
        return Constants.kIdleAngle;
        
      default:
        System.out.println("CODE ERROR! INVALID POSITION! CHECK ARMSUBSYSTEM!");
        return 0;
    }
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
