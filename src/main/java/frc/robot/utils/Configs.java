package frc.robot.utils;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.Encoder;

public class Configs {

  public static CANSparkMax NEO(int deviceID, boolean inverted) {
    CANSparkMax motor = new CANSparkMax(deviceID, MotorType.kBrushless);
    motor.clearFaults();
    motor.setSmartCurrentLimit(40);
    motor.setSecondaryCurrentLimit(40);
    motor.setIdleMode(IdleMode.kBrake);
    motor.setInverted(inverted);
    return motor;
  }

  public static CANSparkMax NEO(int deviceID, boolean inverted, CANSparkMax leader) {
    CANSparkMax motor = NEO(deviceID, inverted);
    motor.follow(leader);
    return motor;
  }

  public static RelativeEncoder RelativeEncoder(CANSparkMax motor, double CF) {
    RelativeEncoder encoder = motor.getEncoder();
    encoder.setPositionConversionFactor(CF);
    encoder.setVelocityConversionFactor(CF / 60);
    return encoder;
  }

  public static Encoder BoreEncoder(int channelA, int channelB, double CF, boolean isInverted) {
    Encoder encoder = new Encoder(channelA, channelB);
    encoder.setReverseDirection(isInverted);
    encoder.setDistancePerPulse(channelB);
    encoder.setDistancePerPulse(CF);
    return encoder;
  }
}
