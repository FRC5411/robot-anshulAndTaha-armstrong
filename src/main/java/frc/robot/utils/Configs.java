package frc.robot.utils;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;

public class Configs {

    public static CANSparkMax NEO(int deviceID, boolean inverted) {
        CANSparkMax motor = new CANSparkMax(deviceID, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.clearFaults();
        motor.setSmartCurrentLimit(40);
        motor.setSecondaryCurrentLimit(40);
        motor.setIdleMode(IdleMode.kBrake);
        motor.setInverted(inverted);
        return motor;
    }

    public static RelativeEncoder RelativeEncoder(CANSparkMax motor, double CF) {
        RelativeEncoder encoder = motor.getEncoder();
        encoder.setPositionConversionFactor(CF);
        encoder.setVelocityConversionFactor(CF / 60);
        return encoder;
    }

    public static RelativeEncoder BoreEncoder(int channelA, int channelB, boolean isInverted) {
        Encoder encoder = new Encoder(channelA, channelB);
        encoder.setReverseDirection(isInverted);
        encoder.setDistancePerPulse(channelB);
        encoder.setPositionConversionFactor(CF);
        encoder.setVelocityConversionFactor(CF / 60);
        return encoder;
    }

}