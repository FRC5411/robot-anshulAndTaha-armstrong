// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class ControllerVars {
    public final static class Constants {

    }

    public final static class Objects {
        public static final CommandXboxController xboxController = new CommandXboxController(0);

        public static final Trigger a = xboxController.a();
        public static final Trigger b = xboxController.b();
        public static final Trigger x = xboxController.x();
        public static final Trigger y = xboxController.y();

        public static final Trigger leftTrigger = xboxController.leftTrigger();
        public static final Trigger rightTrigger = xboxController.rightTrigger();

        public static final Trigger leftBumper = xboxController.leftBumper();
        public static final Trigger rightBumper = xboxController.rightBumper();

        public static final CommandGenericHID buttonBoard = new CommandGenericHID(1);

        public static final Trigger armHigh = buttonBoard.button(1);
        public static final Trigger armMid = buttonBoard.button(2);
        public static final Trigger armLow = buttonBoard.button(3);

        public static final Trigger toggleMode = buttonBoard.button(5);
        
        public static final Trigger armGround = buttonBoard.button(7);
        public static final Trigger armSub = buttonBoard.button(8);

        public static final Trigger armIdle = buttonBoard.button(9);

        public static final Trigger holdSniper = buttonBoard.button(4);

        public static final Trigger armOut = buttonBoard.button(10);
        public static final Trigger armIn = buttonBoard.button(11);

        public static final Trigger intakeButton = buttonBoard.button(12);
        public static final Trigger outtakeButton = buttonBoard.button(6);
    }
}
