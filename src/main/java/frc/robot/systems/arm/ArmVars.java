// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.utils.Configurations;

public class ArmVars {

    /** Variables that do not ever change */
    public final static class Constants {
        /* Motor Constants */
        public static final int ARM_ID = 21;
        public static final int[] ENCODER_PORTS = { 0, 1 };

        /* Motor Adjustments */
        public static final int ARM_CURRENT_LIMIT = 80;

        public static final IdleMode ARM_IDLE_MODE = IdleMode.kBrake;
        public static final boolean ARM_INVERTED = false;

        public static final double ARM_ENCODER_DISTANCE_PER_PULSE = 2.0 * Math.PI / 4096;

        /* Physical properties */
        public static final double ARM_ACCEL_PROFILE = 100.0;//100.0 / 2.0;
        public static final double ARM_VEL_PROFILE = 250.0; //250.0 / 2.0;

        public static final double ARM_GEAR_RATIO = 0.25 / 1.0;

        /* Arm setpoints */

        /** Arm position setpoints for cone */
        public final static class ConeSetpoints {
            public static final double HIGH = Units.degreesToRadians(172.0);// 172.0;
            public static final double MID = 193.0;
            public static final double LOW = 112.0;
            public static final double SUB = 174.0;
            public static final double GRND = 257.0;
        }

        /** Arm position setpoints for cube */
        public static final class CubeSetpoints {
            public static final double HIGH = 173.0;
            public static final double MID = 142.0;
            public static final double LOW = 112.0;
            public static final double SUB = 178.0;
            public static final double GRND = 263.0;
        }

        public static final double SETPOINTS_IDLE = 0.0;
        public static final double SETPOINTS_FLAT = 33.43;

        /* Controller constants */
        public static final double CONTROLLER_P = 1.0;//0.066;
        public static final double CONTROLLER_I = 0.0;
        public static final double CONTROLLER_D = 0.0;

        public static final double CONTROLLER_TOLERANCE = 2.0;

        public static final double FEEDFORWARD_S = 0.0;
        public static final double FEEDFORWARD_G = 0.03;
        public static final double FEEDFORWARD_A = 0.0;
        public static final double FEEDFORWARD_V = 0.0;
    }

    /** Variables that may change, this should be used spareingly */
    public final static class Vars {
        // NOTE: True by default
        public static boolean IS_CONE = true;

        public static boolean IS_SNIPER = false;
    }

    /**
     * Objects that the subsystem uses, such as motors, position estimators, etc.
     */
    public final static class Objects {
        /* Physical objects */
        public static final CANSparkMax ARM_MOTOR = Configurations.SparkMax(Constants.ARM_ID, MotorType.kBrushless,
                Constants.ARM_INVERTED, Constants.ARM_CURRENT_LIMIT, Constants.ARM_IDLE_MODE, 1.0);
        public static final Encoder ARM_ENCODER = Configurations.Encoder(Constants.ENCODER_PORTS[0],
                Constants.ENCODER_PORTS[1], Constants.ARM_ENCODER_DISTANCE_PER_PULSE, false);

        /* Arm controllers */
        public static final TrapezoidProfile.Constraints ARM_CONSTRAINTS = new TrapezoidProfile.Constraints(Constants.ARM_VEL_PROFILE, Constants.ARM_ACCEL_PROFILE);

        public static final ProfiledPIDController ARM_CONTROLLER = new ProfiledPIDController(Constants.CONTROLLER_P, Constants.CONTROLLER_I, Constants.CONTROLLER_D, ARM_CONSTRAINTS);

        public static final ArmFeedforward ARM_FEEDFORWARD = new ArmFeedforward(Constants.FEEDFORWARD_S, Constants.FEEDFORWARD_G, Constants.FEEDFORWARD_V, Constants.FEEDFORWARD_A);
    }

    /** Objects and variables used for simulation */
    public final static class Simulation {
        /* Simulation runtime */
        public static double SIM_LAST_TIME = 0.0;

        /* Simulation physics */
        public static final DCMotor SIM_ARM_GEARBOX = DCMotor.getNEO(1);
        public static final double SIM_ARM_REDUCTION = 192.0;
        public static final double SIM_ARM_MASS = Units.lbsToKilograms(16.0);
        public static final double SIM_ARM_LENGTH = Units.inchesToMeters(31.0);

        /* Simulation objects */
        public static final SingleJointedArmSim SIM_ARM = new SingleJointedArmSim(
                SIM_ARM_GEARBOX,
                SIM_ARM_REDUCTION,
                SingleJointedArmSim.estimateMOI(SIM_ARM_LENGTH, SIM_ARM_MASS),
                SIM_ARM_LENGTH,
                Units.degreesToRadians(-30.0),
                Units.degreesToRadians(225),
                true,
                VecBuilder.fill(Constants.ARM_ENCODER_DISTANCE_PER_PULSE));

        public static final EncoderSim SIM_ARM_ENCODER = new EncoderSim(Objects.ARM_ENCODER);

        /* Simulation visualiser */
        public static final Mechanism2d SIM_ARM_MECH2D = new Mechanism2d(60.0, 60.0);
        public static final MechanismRoot2d SIM_ARM_PIVOT = SIM_ARM_MECH2D.getRoot(
                "ArmPivot", 30.0, 21);
        public static final MechanismLigament2d SIM_ARM_TOWER = SIM_ARM_PIVOT.append(
                new MechanismLigament2d("ArmTower", 21, -90));

        /* Complete visualiser */
        public static final MechanismLigament2d SIM_ARM_VISUAL_MECH2D = SIM_ARM_PIVOT.append(
                new MechanismLigament2d(
                        "Arm",
                        Units.metersToInches(SIM_ARM_LENGTH),
                        Units.radiansToDegrees(SIM_ARM.getAngleRads()),
                        6,
                        new Color8Bit(Color.kWhite)));
    }
}
