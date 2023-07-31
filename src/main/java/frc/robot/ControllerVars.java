// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
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
    }
}
