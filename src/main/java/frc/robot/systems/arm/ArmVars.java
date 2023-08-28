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
        public static final int kArmID = 21; 
        public static final int[] kArmEncoderIDs = {0, 1};
        
        public static final double kArmAcceleration = 100 / 2;
        public static final double kArmVelocity = 250 / 2;

        public static final double kArmGearRatio = 22.755;
        public static final double kSpeedPercentage = 80;
        
        public static final double kIdleAngle = 1;
        public static final double kFlatAngle = 0;

        
        // NOTE: ANGLES ARE OFF - DO NOT RUN
         
        public final static class CubeAngles{
            public static final double kHigh = 160;
            public static final double kMid = 142;
            public static final double kLow = 112;
            public static final double kGround = 263;
            public static final double kSubstation = 178;
        }

        public final static class ConeAngles {
            public static final double kHigh = 172;
            public static final double kMid = 193;
            public static final double kLow = 112;
            public static final double kGround = 257;
            public static final double kSubstation = 174;
        }
    }

    public final static class Objects {
        public static final CANSparkMax armMotor = new CANSparkMax(Constants.kArmID, MotorType.kBrushless); 
        public static final Encoder armBoreEncoder = new Encoder(Constants.kArmEncoderIDs[0], Constants.kArmEncoderIDs[1]);
    }
}
