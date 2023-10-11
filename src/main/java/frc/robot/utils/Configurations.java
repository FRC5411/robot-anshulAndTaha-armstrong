// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.Encoder;

/**
 * A class to configure motors and encoders. Helps reduce repetative code, and should be used to
 * standardise motor and encoder instantiation.
 *
 * <p>Only supports the following: CANSparkMax, RelativeEncoder, Encoder, CANCoder
 */
public class Configurations {
  /**
   * Configures the CANSparkMax motor controller based on the params
   *
   * @param motorID Motor ID on the PDH
   * @param motorType Motor type, brushed or brushless
   * @param inverted Whether or not to invert the motor outputs, only applies if you are using
   *     MotorControllerGroup
   * @param smartCurrentLimit Sets the smart current limit of the motor in amps
   * @param idleMode Sets the idle mode of the motor, either Brake or Coast
   * @param voltageCompensation What to set the voltage compensation to
   * @return The configured motor
   */
  public static CANSparkMax SparkMax(
      int motorID,
      MotorType motorType,
      boolean inverted,
      int smartCurrentLimit,
      IdleMode idleMode,
      double voltageCompensation) {

    CANSparkMax motor = new CANSparkMax(motorID, motorType);

    motor.clearFaults();
    motor.setInverted(inverted);
    motor.setSmartCurrentLimit(smartCurrentLimit);
    motor.setSecondaryCurrentLimit(smartCurrentLimit);
    motor.setIdleMode(idleMode);
    motor.enableVoltageCompensation(voltageCompensation);
    motor.burnFlash();

    return motor;
  }

  /**
   * Configures the built-in CANSparkMax encoder based on the params
   *
   * @param motor Motor that's encoder is being configured
   * @param positionConversionFactor Encoder's position conversion factor
   * @param velocityConversionFactor Encoder's velocity conversion factor
   * @param isInverted If encoder should be inverted
   * @return the configured encoder
   */
  public static RelativeEncoder SparkMaxRelativeEncoder(
      CANSparkMax motor, double positionConversionFactor, double velocityConversionFactor) {

    RelativeEncoder encoder = motor.getEncoder();

    encoder.setPosition(0.0);

    encoder.setPositionConversionFactor(positionConversionFactor);
    encoder.setVelocityConversionFactor(velocityConversionFactor);

    return encoder;
  }

  /**
   * Configures a through-bore encoder based on the params
   *
   * @param channelA The A DIO channel of the encoder
   * @param channelB The B DIO channel of the encoder
   * @param conversionFactor The distance per pulse conversion factor
   * @param inverted If the encoder's measuring should be inverted
   * @return The configured encoder
   */
  public static Encoder Encoder(
      int channelA, int channelB, double conversionFactor, boolean inverted) {
    Encoder encoder = new Encoder(channelA, channelB);

    encoder.setDistancePerPulse(conversionFactor);
    encoder.setReverseDirection(inverted);

    return encoder;
  }

  /**
   * A few basic configurations for initializing a CANCoder
   *
   * @param encoderID The ID of the encoder
   * @param sensorRange The absolute sensor range
   * @param inverted The sensor direction (inverted or not)
   * @param initStrat Encoder's initialization configs
   * @return The configured encoder
   */
  public static CANCoder CANCoder(
      int encoderID,
      AbsoluteSensorRange sensorRange,
      boolean inverted,
      SensorInitializationStrategy initStrat) {
    CANCoder encoder = new CANCoder(encoderID);

    encoder.configFactoryDefault();
    encoder.configAbsoluteSensorRange(sensorRange);
    encoder.configSensorDirection(inverted);
    encoder.configSensorInitializationStrategy(initStrat);

    return encoder;
  }

  /**
   * Configures all settings for a CANCoder
   *
   * @param encoderID The ID of the encoder
   * @param configurations Configurations object
   * @return The configured encoder
   */
  public static CANCoder CANCoder(int encoderID, CANCoderConfiguration configurations) {
    CANCoder encoder = new CANCoder(encoderID);

    encoder.configFactoryDefault();
    encoder.configAllSettings(configurations);

    return encoder;
  }
}
