package frc.robot.systems.drive;
import frc.robot.utils.Configs;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class DriveVars {
    public final static class Constants {
        public static final int kLeftFrontID = 11;
        public static final int kLeftBackID = 12;
        public static final int kRightFrontID = 13;
        public static final int kRightBackID = 14;

        public static final boolean kLeftInvert = true;
        public static final boolean kRightInvert = false;

        public static final double kWheelRadiusMeters = 0.0762;
        public static final double kGearRatio = 7.89;

        public static final double ConversionFactor = (2 * Math.PI * kWheelRadiusMeters) / kGearRatio;
        public static final double SF = (2.16/0.548);
        public static final double ScaledCF = ConversionFactor * SF;
    }

    public final static class Objects {
        CANSparkMax leftFront = Configs.NEO(Constants.kLeftFrontID, Constants.kLeftInvert);
        CANSparkMax leftBack = Configs.NEO(Constants.kLeftBackID, Constants.kLeftInvert);
        CANSparkMax rightFront = Configs.NEO(Constants.kRightFrontID, Constants.kRightInvert);
        CANSparkMax rightBack = Configs.NEO(Constants.kRightBackID, Constants.kRightInvert);

        RelativeEncoder leftFrontEncoder = Configs.RelativeEncoder(leftFront, Constants.ScaledCF);
        RelativeEncoder leftBackEncoder = Configs.RelativeEncoder(leftBack, Constants.ScaledCF);
        RelativeEncoder rightFrontEncoder = Configs.RelativeEncoder(rightFront, Constants.ScaledCF);
        RelativeEncoder rightBackEncoder = Configs.RelativeEncoder(rightBack, Constants.ScaledCF);
    }


}
