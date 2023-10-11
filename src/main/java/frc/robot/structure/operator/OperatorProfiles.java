// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.structure.operator;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.structure.operator.ButtonMap.Objects;
import frc.robot.systems.drive.DriveSubsystem;
import frc.robot.systems.drive.DriveVars.Vars;
import frc.robot.utils.PilotProfile;

/** Class to store all operator profiles */
public class OperatorProfiles {

  public interface Profile {
    public double deadzones = 0.0;
    public boolean squareInputs = false;

    public PilotProfile profile = null;

    /**
     * Configures controller bindings for the driver as well as preferred settings
     *
     * @param robotDrive Required subsystem
     */
    public default void configureBindings(DriveSubsystem robotDrive) {}
  }

  /** System default settings */
  public static final class SystemDefault implements Profile {
    /* Preference and binding names */
    public final String NAME = "SYSTEM DEFAULT";
    public final String DEADZONE = "DEADZONE";
    public final String SQUARE_INPUTS = "SQUAREINPUTS";
    public final String B_RESET_GYRO = "RESET GYRO";

    /* Edit Preferences Here */
    private final double deadzones = 0.0;
    private final boolean squareInputs = false;

    /* Create profile */
    public PilotProfile profile =
        new PilotProfile(NAME)
            .addPreference(DEADZONE, () -> deadzones)
            .addPreference(SQUARE_INPUTS, () -> squareInputs)
            .addKeybind(B_RESET_GYRO, Objects.DRIVER_CONTROLLER.y());

    @Override
    public void configureBindings(DriveSubsystem robotDrive) {
      profile
          .getKeybind(B_RESET_GYRO)
          .onTrue(Commands.runOnce(robotDrive::resetGyroYaw, robotDrive));
      Vars.DEADZONE = (Double) profile.getPreference(DEADZONE);
      Vars.SQUARE_INPUTS = (Boolean) profile.getPreference(SQUARE_INPUTS);
    }
  }

  /** Aaron's settings */
  public static final class Aaron implements Profile {
    /* Preference and binding names */
    public final String NAME = "AARON";
    public final String DEADZONE = "DEADZONE";
    public final String SQUARE_INPUTS = "SQUAREINPUTS";
    public final String B_RESET_GYRO = "RESET GYRO";

    /* Edit Preferences Here */
    private final double deadzones = 0.1;
    private final boolean squareInputs = true;

    /* Create profile */
    public PilotProfile profile =
        new PilotProfile(NAME)
            .addPreference(DEADZONE, () -> deadzones)
            .addPreference(SQUARE_INPUTS, () -> squareInputs)
            .addKeybind(B_RESET_GYRO, Objects.DRIVER_CONTROLLER.y());

    @Override
    public void configureBindings(DriveSubsystem robotDrive) {
      profile
          .getKeybind(B_RESET_GYRO)
          .onTrue(Commands.runOnce(robotDrive::resetGyroYaw, robotDrive));
      Vars.DEADZONE = (Double) profile.getPreference(DEADZONE);
      Vars.SQUARE_INPUTS = (Boolean) profile.getPreference(SQUARE_INPUTS);
    }
  }
}
