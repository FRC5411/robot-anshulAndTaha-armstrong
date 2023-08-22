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
    public final static class Constants {
        // Motor and Tank constants
        public static final int kLeftFrontID = 11;
        public static final int kLeftBackID = 12;
        public static final int kRightFrontID = 13;
        public static final int kRightBackID = 14;

        public static final boolean kLeftInvert = false;
        public static final boolean kRightInvert = false;

        public static final double kWheelRadiusMeters = Units.inchesToMeters(3);
        public static final double kGearRatio = 7.89;

        public static final double kMassKg = 125;

        // Arbitrary
        public static final double kMOIKGMeterSquared = 10;

        // Odometry
        // public static final double kConversionFactor = (2 * Math.PI * kWheelRadiusMeters) / kGearRatio;
        public static final double kConversionFactor = Units.inchesToMeters(1 / (kGearRatio * 2 * Math.PI * kWheelRadiusMeters));
        public static final double kSF = (2.16/0.548);
        public static final double kScaledCF = kConversionFactor * kSF;

        // Path Planner
        public static final double kTrackWidthMeters = Units.inchesToMeters(21); // 21

        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7; 
        public static final DifferentialDriveKinematics kTankKinematics = 
                            new DifferentialDriveKinematics(kTrackWidthMeters);

        public static final double kVolts = 0.14592; //kS
        public static final double kVoltsMPS = 2.0809; //kV
        public static final double kVoltsMSPS = 0.83925; //kA

        public static final double kPDrive = 1.4;
        public static final double kIDrive = 0;
        public static final double kDDrive = 0;

        // Deadzones
        public static final double kDeadzone = 0.1;
        public static final double kRotationScaler = 0.6;

        // Vision
        public static final Vector<N3> kVisionMeasurementStdDevs = VecBuilder.fill(0.1, 0.1, Math.toRadians(200));

        public static final double kSniperScaler = 0.4;
    }

    public final static class Objects {
        public static final CANSparkMax leftFront = Configs.NEO(Constants.kLeftFrontID, Constants.kLeftInvert);
        public static final CANSparkMax leftBack = Configs.NEO(Constants.kLeftBackID, Constants.kLeftInvert, leftFront);
        public static final CANSparkMax rightFront = Configs.NEO(Constants.kRightFrontID, Constants.kRightInvert);
        public static final CANSparkMax rightBack = Configs.NEO(Constants.kRightBackID, Constants.kRightInvert, rightFront);

        public static final RelativeEncoder leftFrontEncoder = Configs.RelativeEncoder(leftFront, Constants.kScaledCF);
        public static final RelativeEncoder leftBackEncoder = Configs.RelativeEncoder(leftBack, Constants.kScaledCF);
        public static final RelativeEncoder rightFrontEncoder = Configs.RelativeEncoder(rightFront, Constants.kScaledCF);
        public static final RelativeEncoder rightBackEncoder = Configs.RelativeEncoder(rightBack, Constants.kScaledCF);

        public static final RamseteController ramseteController = new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta);

        public static final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
            Constants.kVolts, Constants.kVoltsMPS, Constants.kVoltsMSPS);

        public static final DifferentialDrive robotDrive = new DifferentialDrive(leftFront, rightFront);

        public static final AHRS navX = new AHRS();

        public static final DifferentialDrivePoseEstimator poseEstimator = new DifferentialDrivePoseEstimator(
            Constants.kTankKinematics, new Rotation2d(), 0, 0, new Pose2d());

        public static final Field2d field = new Field2d();
    }

    public final static class Simulation {
        public static double lasttime = 0;

        public static final Field2d field = new Field2d();

        PWMSparkMax m_leftFront = new PWMSparkMax(Constants.kLeftFrontID);
        PWMSparkMax m_leftBack = new PWMSparkMax(Constants.kLeftBackID);
        
        public static final LinearSystem<N2, N2, N2> m_drivetrainSystem =
        LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5, 0.3);

        public static final DifferentialDrivetrainSim driveSim = new DifferentialDrivetrainSim(
            DCMotor.getNEO(2),
            Constants.kGearRatio,
            Constants.kMassKg,
            Constants.kMOIKGMeterSquared,
            Constants.kWheelRadiusMeters,
            Constants.kTrackWidthMeters,
            VecBuilder.fill(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
    }
}