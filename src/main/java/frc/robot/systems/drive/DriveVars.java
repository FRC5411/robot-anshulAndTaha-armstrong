package frc.robot.systems.drive;

import frc.robot.utils.Configs;

import com.kauailabs.navx.frc.AHRS;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class DriveVars {
    /**
     * Variables that do not ever change
     */
    public final static class Constants {
        /* Motor constants */
        public static final int LEFT_FRONT_ID = 11;
        public static final int LEFT_BACK_ID = 12;
        public static final int RIGHT_FRONT_ID = 13;
        public static final int RIGHT_BACK_ID = 14;

        public static final boolean LEFT_INVERT = false;
        public static final boolean RIGHT_INVERT = true;

        /* Motor adjustments */
        public static final double SPEED_SCALER = 0.4;
        public static final double ROTATION_SCALER = 0.6;

        /* Physical properties */
        public static final double WHEEL_RADIUS_M = 0.0762;
        public static final double TRACK_WIDTH_M = Units.inchesToMeters(21);

        public static final double GEAR_RATIO = 7.89;

        public static final double MASS_KG = 125;

        public static final double kMOIKGMeterSquared = 10;
        public static final double SCALED_CF = Units.inchesToMeters(1 / GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS_M)
                * (2.16 / 0.548);

        /* Kinematics */
        public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(
                TRACK_WIDTH_M);

        /* Path planner constants */
        public static final double RAMSETE_B = 2;
        public static final double RAMSETE_ZETA = 0.7;

        /* Controller constants */
        public static final double DRIVE_VOLTS = 0.14592; // kS
        public static final double DRIVE_VOLTS_MPS = 2.0809; // kV
        public static final double DRIVE_VOLTS_MSPS = 0.83925; // kA

        public static final double DRIVE_PP_P = 0.0012;
        public static final double DRIVE_PP_I = 0;
        public static final double DRIVE_PP_D = 0;

        // Vision
        public static final Vector<N3> VISION_MEASUREMENT_STD_DEV = VecBuilder.fill(0.1, 0.1, Math.toRadians(200));
    }

    /**
     * Variables that may change, this should be used spareingly
     */
    public static class Vars {
        /* Driver preferences */
        public static boolean SQUARE_INPUTS = true;
        public static double DEADZONE = 0.0;
    }

    /**
     * Objects that the subsystem uses, such as motors, position estimators, etc.
     */
    public final static class Objects {
        /* Motors */
        public static final CANSparkMax LEFT_FRONT_MOTOR = Configs.NEO(Constants.LEFT_FRONT_ID, Constants.LEFT_INVERT);
        public static final CANSparkMax LEFT_BACK_MOTOR = Configs.NEO(Constants.LEFT_BACK_ID, Constants.LEFT_INVERT,
                LEFT_FRONT_MOTOR);
        public static final CANSparkMax RIGHT_FRONT_MOTOR = Configs.NEO(Constants.RIGHT_FRONT_ID,
                Constants.RIGHT_INVERT);
        public static final CANSparkMax RIGHT_BACK_MOTOR = Configs.NEO(Constants.RIGHT_BACK_ID, Constants.RIGHT_INVERT,
                RIGHT_FRONT_MOTOR);

        /* Encoders */
        public static final RelativeEncoder LEFT_FRONT_ENCODER = Configs.RelativeEncoder(LEFT_FRONT_MOTOR,
                Constants.SCALED_CF);
        public static final RelativeEncoder LEFT_BACK_ENCODER = Configs.RelativeEncoder(LEFT_BACK_MOTOR,
                Constants.SCALED_CF);
        public static final RelativeEncoder RIGHT_FRONT_ENCODER = Configs.RelativeEncoder(RIGHT_FRONT_MOTOR,
                Constants.SCALED_CF);
        public static final RelativeEncoder RIGHT_BACK_ENCODER = Configs.RelativeEncoder(RIGHT_BACK_MOTOR,
                Constants.SCALED_CF);

        /* Controllers */
        public static final RamseteController DRIVE_RAMSETE_CONTROLLER = new RamseteController(Constants.RAMSETE_B,
                Constants.RAMSETE_ZETA);

        public static final SimpleMotorFeedforward DRIVE_FEEDFORWARD = new SimpleMotorFeedforward(
                Constants.DRIVE_VOLTS, Constants.DRIVE_VOLTS_MPS, Constants.DRIVE_VOLTS_MSPS);

        /* Drive */
        public static final DifferentialDrive ROBOT_DRIVE = new DifferentialDrive(LEFT_FRONT_MOTOR, RIGHT_FRONT_MOTOR);

        /* Odometry */
        public static final AHRS NAVX = new AHRS();

        public static final DifferentialDrivePoseEstimator ROBOT_POSE_ESTIMATOR = new DifferentialDrivePoseEstimator(
                Constants.DRIVE_KINEMATICS, new Rotation2d(), 0, 0, new Pose2d());

        public static final Field2d ROBOT_FIELD = new Field2d();
    }

    /**
     * Objects and variables used for simulation
     */
    public final static class Simulation {
        public static double lasttime = 0;

        PWMSparkMax leftFrontMotorSIM = new PWMSparkMax(Constants.LEFT_FRONT_ID);
        PWMSparkMax leftBackMotorSIM = new PWMSparkMax(Constants.LEFT_BACK_ID);

        public static final LinearSystem<N2, N2, N2> DRIVE_SYSTEM = LinearSystemId.identifyDrivetrainSystem(1.98, 0.2,
                1.5, 0.3);

        public static final DifferentialDrivetrainSim DRIVE_SIMULATOR = new DifferentialDrivetrainSim(
                DCMotor.getNEO(2),
                Constants.GEAR_RATIO,
                Constants.MASS_KG,
                Constants.kMOIKGMeterSquared,
                Constants.WHEEL_RADIUS_M,
                Constants.TRACK_WIDTH_M,
                VecBuilder.fill(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
    }
}