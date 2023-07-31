// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Encoder;

/** Add your docs here. */
public class ArmVars {
    public final static class Constants {
        public static final int kArmID = 1; 
        public static final int[] kArmEncoderIDs = {0, 1};
        
        public static final double kArmAcceleration = 100;
        public static final double kArmVelocity = 250;

        public static final double kArmGearRatio = 22.755;
        public static final double kSpeedPercentage = 60;
        
        public static final double kIdleAngle = 0;
        public static final double kFlatAngle = 0;

        public final static class CubeAngles{
            public static final double kHigh = 0;
            public static final double kMid = 0;
            public static final double kLow = 0;
            public static final double kGround = 0;
            public static final double kSubstation = 0;
        }

        public final static class ConeAngles {
            public static final double kHigh = 0;
            public static final double kMid = 0;
            public static final double kLow = 0;
            public static final double kGround = 0;
            public static final double kSubstation = 0;
        }
    }

    public final static class Objects {
        public static final CANSparkMax armMotor = new CANSparkMax(Constants.kArmID, MotorType.kBrushless); 
        public static final Encoder armBoreEncoder = new Encoder(Constants.kArmEncoderIDs[0], Constants.kArmEncoderIDs[1]);
    }
}
