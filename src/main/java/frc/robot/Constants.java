// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/** Add your docs here. */
public class Constants {
    public static final Mode currentMode = Mode.SIM;
    private static final RobotType robot = RobotType.ROBOT_SIMBOT;

    public static enum Mode {
      /** Running on a real robot. */
      REAL,
  
      /** Running a physics simulator. */
      SIM,
  
      /** Replaying from a log file. */
      REPLAY
    }

    public static RobotType getRobot() {
        if (RobotBase.isReal()) {
          if (robot == RobotType.ROBOT_SIMBOT) { // NOTE: Invalid robot selected
            return RobotType.ROBOT_2023S;
          } else {
            return robot;
          }
        } else {
          return robot;
        }
      }
    
      public static Mode getMode() {
        switch (getRobot()) {
          case ROBOT_2023S:
            return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
    
          case ROBOT_SIMBOT:
            return Mode.SIM;
    
          default:
            return Mode.REAL;
        }
      }

      public static enum RobotType {
        ROBOT_2023S, ROBOT_SIMBOT
      }
}
