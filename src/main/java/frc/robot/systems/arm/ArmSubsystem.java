// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotStates;
import frc.robot.systems.arm.ArmVars.Constants;

public class ArmSubsystem extends SubsystemBase {
  private ArmIO armIO;

  public ArmSubsystem() {
    armIO = new ArmIO();
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////

  public void setArmTeleop(double speed){
    armIO.setArmReduc(speed);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////

  // NOTE: This function uses isACone as a paramater and not a global variable because we don't want it to affect buttonboard
  public FunctionalCommand armPIDAuton(ArmSubsystem robotArm, String strSetpoint, boolean isACone){
    double kP = 0.066;
    double kI = 0;
    double kD = 0;

    ProfiledPIDController pid = new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(Constants.kArmVelocity, Constants.kArmAcceleration));
    
    return new FunctionalCommand(() -> {

      // Initialize Code
        pid.setTolerance(2);
        pid.reset(ArmIO.getArmAngle());

        System.out.println("Command AUTON ARM ALIGN has started");
      }, () -> {

      // Execute Code
        double calc = pid.calculate(ArmIO.getArmAngle(), returnAngle(strSetpoint, isACone));
        armIO.setArm(calc);
      }, 

      interrupted -> {}, 

      () -> false, 
      
      robotArm);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////

  public FunctionalCommand armFFHold(ArmSubsystem robotArm) {
    double kg = 0.03;

    ArmFeedforward feedForward = new ArmFeedforward(0, kg, 0, 0);
    
    return new FunctionalCommand(
      // Init
      () -> System.out.println("Command HOLD ARM COMMAND has started"),
      // Exec
      () -> {
        if(RobotStates.sShouldHoldArm){
          double calc = feedForward.calculate(ArmIO.getXAxisArmAngle(), 0);
    
          armIO.setArm(calc);

          System.out.println("\nAS : EXECUTING armFFHold" + "\nAS : ARM CALC " + calc + "\nAS : ARM ANGLE " + ArmIO.getArmAngle());
        }}, 
      interrupted -> {},
      () -> false, 
      robotArm);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////

  public FunctionalCommand armPIDTeleop(ArmSubsystem robotArm, String strSetpoint){
    double kP = 0.066;
    double kI = 0;
    double kD = 0;

    ProfiledPIDController pid = new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(Constants.kArmVelocity, Constants.kArmAcceleration));
    
    return new FunctionalCommand(() -> {

      // Init
        pid.setTolerance(2);
        pid.reset(ArmIO.getArmAngle());

        System.out.println("Command TELEOP ARM ALIGN has started");
      }, () -> {

      // Exec
        double calc = pid.calculate(ArmIO.getArmAngle(), returnAngle(strSetpoint, RobotStates.sIsConeMode));
        
        armIO.setArm(calc);

        System.out.println("\n\nAS : EXECUTING armPIDTeleop" + "\nAS : ARM CALC " + calc + "\nAS : ARM ANGLE " + ArmIO.getArmAngle() + "\nAS : RETURNED ANGLE " + returnAngle(strSetpoint, RobotStates.sIsConeMode));
      }, 

      interrupted -> {}, 

      () -> false, 
      
      robotArm);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////

  private double returnAngle(String pos, boolean coneState){
    switch(pos){
      case "high":
        return (coneState) ? Constants.ConeAngles.kHigh : Constants.CubeAngles.kHigh;

      case "mid":
        return (coneState) ? Constants.ConeAngles.kMid : Constants.CubeAngles.kMid;
        
      case "low":
        return (coneState) ? Constants.ConeAngles.kLow : Constants.CubeAngles.kLow;
        
      case "ground":
        return (coneState) ? Constants.ConeAngles.kGround : Constants.CubeAngles.kGround;
        
      case "substation":
        return (coneState) ? Constants.ConeAngles.kSubstation : Constants.CubeAngles.kSubstation;

      case "idle":
        return Constants.kIdleAngle;
        
      default:
        System.out.println("CODE ERROR! INVALID POSITION! CHECK ARMSUBSYSTEM!");
        return 0;
    }
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("ARM POITION : ", ArmIO.getArmAngle());
  }
}
