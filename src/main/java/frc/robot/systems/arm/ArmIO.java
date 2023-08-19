// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.arm;

import frc.robot.systems.arm.ArmVars.Constants;
import frc.robot.systems.arm.ArmVars.Objects;
import frc.robot.utils.Conversions;

public class ArmIO {
    public ArmIO() {}
    
    public static double getArmAngle(){
        return Conversions.boreEncTicksToDeg(Objects.armBoreEncoder.getDistance(), Constants.kArmGearRatio);
    }

    // WARNING: Limits for arm will not work if setArm isn't being updated periodically
    public void setArm(double speed){
        if(safeToMoveArm(speed)) { Objects.armMotor.set(speed); }

        System.out.println("\nAI : safeToMoveArm " + safeToMoveArm(speed));
    }

    // WARNING: Limits for arm will not work if setArm isn't being updated periodically
    public void setArmReduc(double speed){
        if(safeToMoveArm(speed)) Objects.armMotor.set(speed * (Constants.kSpeedPercentage * 0.01));
    }

    public static double getXAxisArmAngle() {
        double ffAngleDegs = getArmAngle() - Constants.kFlatAngle;

        if(ffAngleDegs < 0) ffAngleDegs += 360;
        
        return Math.toRadians(ffAngleDegs);
    }

    public boolean safeToMoveArm(double desiredSpeed) {
        double armPosition = getArmAngle();

        if ((armPosition > 263 && desiredSpeed > 0) || 
            (armPosition < 3 && desiredSpeed < 0)) {
            setArm(0);
            return false;
        };

        return true;
      }

}
