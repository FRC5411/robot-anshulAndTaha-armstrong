// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.structure.operator;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Class to hold a map of the buttons */
public class ButtonMap {

  /** Map of controller IDs */
  public static class Map {
    /* Ports */
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    /* Arm Poses */
    public static final int B_ARM_HIGH = 1;
    public static final int B_ARM_MID = 2;
    public static final int B_ARM_LOW = 3;

    public static final int B_ARM_GROUND = 7;
    public static final int B_ARM_SUB = 8;

    public static final int B_ARM_IDLE = 9;

    /* Toggle game mode */
    public static final int B_TOGGLE_MODE = 5;

    /* Manual controls */
    // NOTE: Sniper only applies when the arm is being moved manually
    public static final int B_ARM_SNIPER = 4;

    public static final int B_ARM_OUT_MANUAL = 10;
    public static final int B_ARM_IN_MANUAL = 11;

    public static final int B_INTAKE_IN = 12;
    public static final int B_INTAKE_OUT = 6;
  }

  /** Class to hold the button and controller objects */
  public static class Objects {
    /* Input objects */
    public static final CommandXboxController DRIVER_CONTROLLER =
        new CommandXboxController(Map.DRIVER_CONTROLLER_PORT);
    public static final CommandGenericHID OPERATOR_CONTROLLER =
        new CommandGenericHID(Map.OPERATOR_CONTROLLER_PORT);

    public static final Trigger B_ARM_HIGH = OPERATOR_CONTROLLER.button(Map.B_ARM_HIGH);
    public static final Trigger B_ARM_MID = OPERATOR_CONTROLLER.button(Map.B_ARM_MID);
    public static final Trigger B_ARM_LOW = OPERATOR_CONTROLLER.button(Map.B_ARM_LOW);

    public static final Trigger B_ARM_GROUND = OPERATOR_CONTROLLER.button(Map.B_ARM_GROUND);
    public static final Trigger B_ARM_SUB = OPERATOR_CONTROLLER.button(Map.B_ARM_SUB);

    public static final Trigger B_ARM_IDLE = OPERATOR_CONTROLLER.button(Map.B_ARM_IDLE);

    public static final Trigger B_TOGGLE_MODE = OPERATOR_CONTROLLER.button(Map.B_TOGGLE_MODE);

    public static final Trigger B_ARM_SNIPER = OPERATOR_CONTROLLER.button(Map.B_ARM_SNIPER);

    public static final Trigger B_ARM_OUT_MANUAL = OPERATOR_CONTROLLER.button(Map.B_ARM_OUT_MANUAL);
    public static final Trigger B_ARM_IN_MANUAL = OPERATOR_CONTROLLER.button(Map.B_ARM_IN_MANUAL);

    public static final Trigger B_INTAKE_IN = OPERATOR_CONTROLLER.button(Map.B_INTAKE_IN);
    public static final Trigger B_INTAKE_OUT = OPERATOR_CONTROLLER.button(Map.B_INTAKE_OUT);
  }
}
