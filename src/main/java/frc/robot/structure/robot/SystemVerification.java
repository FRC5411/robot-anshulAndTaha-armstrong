// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.structure.robot;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.Constants.RobotType;
import frc.robot.systems.arm.ArmIOSpM;
import frc.robot.systems.arm.ArmSubsystem;
import frc.robot.systems.drive.DriveIO;
import frc.robot.systems.drive.DriveIOSim;
import frc.robot.systems.drive.DriveIOSpM;
import frc.robot.systems.drive.DriveSubsystem;
import frc.robot.systems.drive.GyroIO;
import frc.robot.systems.drive.NavXIO;

/** Class to check which system the robot is running on */
public class SystemVerification {

    /** Creates a new System Verification object */
    public SystemVerification() {
    }

    /**
     * Verifies which mode the robot is in, then checks to see if it is real
     * or simulated. Then sets the subsystem accordingly
     * 
     * @return The correct DriveSubsystem for the robot
     */
    public DriveSubsystem verifyRobotDrive() {
        /* If robot is not a replay */
        if (getMode() != Mode.REPLAY) {
            switch (getRobot()) {
                /* If robot is a real robot */
                case ROBOT_2023S:
                    return new DriveSubsystem(new DriveIOSpM(), new NavXIO());
                /* If robot is simulating */
                case ROBOT_SIMBOT:
                    return new DriveSubsystem(new DriveIOSim(), new GyroIO() {
                    });
                default:
                    break;
            }
        }
        /* If robot is in fact in replay mode */
        return new DriveSubsystem(new DriveIO() {
        }, new GyroIO() {
        });
    }

    /**
     * Verifies which mode the robot is in, then checks to see if it is real
     * or simulated. Then sets the subsystem accordingly
     * 
     * @return The correct ArmSubsystem for the robot
     */
    public ArmSubsystem verifyRobotArm() {
        /* If robot is not a replay */
        if (getMode() != Mode.REPLAY) {
            switch (getRobot()) {
                /* If robot is a real robot */
                case ROBOT_2023S:
                    return new ArmSubsystem(new ArmIOSpM());
                /* If robot is simulating */
                case ROBOT_SIMBOT:
                    return new ArmSubsystem(new ArmIOSpM());
                default:
                    break;
            }
        }
        /* If robot is in fact in replay mode */
        return new ArmSubsystem(new ArmIOSpM());
    }

    /**
     * Gets if robot is real or is simulated
     * 
     * @return What robot it is
     */
    private static RobotType getRobot() {
        if (RobotBase.isReal()) {
            if (Constants.ROBOT == RobotType.ROBOT_SIMBOT) { // NOTE: Invalid robot selected
                return RobotType.ROBOT_2023S;
            } else {
                return Constants.ROBOT;
            }
        }
        return Constants.ROBOT;
    }

    /**
     * Gets what mode the robot is currently in
     * 
     * @return Current robot mode
     */
    private static Mode getMode() {
        switch (getRobot()) {
            case ROBOT_2023S:
                return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;

            case ROBOT_SIMBOT:
                return Mode.SIM;

            default:
                return Mode.REAL;
        }
    }
}
