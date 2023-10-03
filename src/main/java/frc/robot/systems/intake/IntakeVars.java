// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.LinearFilter;
import frc.robot.utils.Configurations;

public class IntakeVars {

    /** Variables that do not ever change */
    public final static class Constants {
        /* Motor Constants */
        public static final int INTAKE_ID = 22;

        /* Motor Adjustments */
        public static final int INTAKE_CURRENT_LIMIT = 20;

        public static final boolean INTAKE_INVERTED = false;
        public static final IdleMode INTAKE_IDLE_MODE = IdleMode.kCoast;

        /* Scalers */
        public static final double PERCENT_CONE_SPEED = 0.5;
        public static final double PERCENT_CUBE_SPEED = 1.0;

        /* Linear Filter */
        public static final int FILTER_SAMPLES = 25;
    }

    /** Variables that may change, this should be used spareingly */
    public static class Vars {
        // NOTE: True by default
        public static boolean IS_CONE = true;
    }

    /**
     * Objects that the subsystem uses, such as motors, position estimators, etc.
     */
    public static class Objects {
        /* Motor */
        public static final CANSparkMax INTAKE_MOTOR = Configurations.SparkMax(Constants.INTAKE_ID,
                MotorType.kBrushless, Constants.INTAKE_INVERTED, Constants.INTAKE_CURRENT_LIMIT,
                Constants.INTAKE_IDLE_MODE, 1.0);

        public static final RelativeEncoder INTAKE_ENCODER = Configurations.SparkMaxRelativeEncoder(INTAKE_MOTOR, 1.0,
                1.0);

        public static LinearFilter INTAKE_LIN_FILTER = LinearFilter.movingAverage(Constants.FILTER_SAMPLES);
    }

    /** Objects and variables used for simulation */
    public final static class Simulation {
    }
}
