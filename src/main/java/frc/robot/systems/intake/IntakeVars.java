// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/** Add your docs here. */
public class IntakeVars {
    public final static class Constants {
        public static final int kIntakeID = 22;

        public static final int kIntakeCurrentLimit = 50;

        public static final double kIntakeConeSpeed = 0.5;
        public static final double kIntakeCubeSpeed = 1.0;
    }

    public final static class Objects {
        public static final CANSparkMax intakeMotor = new CANSparkMax(Constants.kIntakeID, MotorType.kBrushless);
    }
}
