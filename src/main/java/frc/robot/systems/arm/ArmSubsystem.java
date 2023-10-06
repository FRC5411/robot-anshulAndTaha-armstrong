// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.systems.arm.ArmVars.Constants;
import frc.robot.systems.arm.ArmVars.Objects;
import frc.robot.systems.arm.ArmVars.Simulation;
import frc.robot.systems.arm.ArmVars.Vars;
import frc.robot.systems.arm.ArmVars.Constants.ConeSetpoints;
import frc.robot.systems.arm.ArmVars.Constants.CubeSetpoints;
import frc.robot.utils.Alert;
import frc.robot.utils.Alert.AlertType;

// TODO: Add soft limits for the arm

/** Arm subsytem class */
public class ArmSubsystem extends SubsystemBase {

    private final ArmIO m_armIO;

    private final ArmIOInputsAutoLogged m_armInputs = new ArmIOInputsAutoLogged();

    /**
     * Creates a new instance of the arm subsystem.
     * 
     * @param armIO Type of IO the subsystem will be using
     */
    public ArmSubsystem(ArmIO armIO) {
        m_armIO = armIO;
    }

    @Override
    public void periodic() {
        /* Update arm inputs */
        m_armIO.updateInputs(m_armInputs);
        Logger.getInstance().processInputs("/systems/arm/armIO", m_armInputs);
    }

    @Override
    public void simulationPeriodic() {
        /* Update simulated inputs */
        m_armIO.updateInputs(m_armInputs);
        Logger.getInstance().processInputs("/systems/arm/armIO", m_armInputs);

        /* Update simulator */
        Simulation.SIM_ARM.update(0.02);

        /* Update encoder */
        // Simulation.SIM_ARM_ENCODER.setDistance(Math.toDegrees(Simulation.SIM_ARM.getAngleRads()));
        Simulation.SIM_ARM_ENCODER.setDistance(Simulation.SIM_ARM.getAngleRads());

        /* Update visualiser */
        // Simulation.SIM_ARM_VISUAL_MECH2D.setAngle(Math.toDegrees(Simulation.SIM_ARM.getAngleRads()));
        Simulation.SIM_ARM_VISUAL_MECH2D.setAngle(Units.radiansToDegrees(Simulation.SIM_ARM.getAngleRads()));

        Logger.getInstance().recordOutput("/systems/arm/simArmMech", Simulation.SIM_ARM_MECH2D);
        SmartDashboard.putData(Simulation.SIM_ARM_MECH2D);

        /* Update time */
        Simulation.SIM_LAST_TIME += 0.02;
    }

    /**
     * Sets the voltage of the arm
     * 
     * @param volts Volts
     */
    public void setSpeeds(double volts) {
        m_armIO.setVolts(volts);
        Logger.getInstance().recordOutput("/systems/arm/setSpeedsVolts", MathUtil.clamp(volts, -12.0, 12.0));
    }

    /**
     * Moves or holds the arm based on setpoints
     * 
     * @param setpoint  Setpoint to move the arm to (Will not do anything if
     *                  isHolding is true)
     * @param isHolding If the arm should hold it's last position
     * @return Command to manipulate the arm based on the params
     */
    public FunctionalCommand MoveArmCommand(String setpoint, boolean isHolding) {
        /* Convert String goal to setpoint based on gamemode */
        double goal = getSetpoint(setpoint);

        /* Set controller tolerance (degs) */
        Objects.ARM_CONTROLLER.setTolerance(Constants.CONTROLLER_TOLERANCE);

        return new FunctionalCommand(
                /* Init Code */
                () -> {
                },
                /* Execute Code */
                () -> {
                    /* Calc percent output vals */
                    double feedforwardCalc = Objects.ARM_FEEDFORWARD.calculate(getFeedForwardAngle(), 0.0);
                    double profileCalc = Objects.ARM_CONTROLLER.calculate(m_armInputs.armPosition, goal);
                    Logger.getInstance().recordOutput("/systems/arm/profileCalc", profileCalc);
                    Logger.getInstance().recordOutput("/systems/arm/goal", goal);

                    /* Set volts based on percent output */
                    if (isHolding) {
                        setSpeeds(feedforwardCalc * 12.0);
                    } else {
                        setSpeeds(profileCalc * 12.0);
                    }
                },
                /* End code */
                interrupted -> {
                    /* Set volts to 0 when command ends */
                    setSpeeds(0.0);
                },
                /* Is finished */
                () -> false,
                /* Required subsystems */
                this);
    }

    /**
     * Gets the correct setpoint for the arm based on the gamepiece
     * 
     * @param desiredSetpoint What position the arm needs to go
     * @return The correct setpoint
     */
    public double getSetpoint(String desiredSetpoint) {
        /* Will switch setpoint based on gamepiece mode */
        switch (desiredSetpoint) {
            case "HIGH":
                return (Vars.IS_CONE) ? ConeSetpoints.HIGH : CubeSetpoints.HIGH;
            case "MID":
                return (Vars.IS_CONE) ? ConeSetpoints.MID : CubeSetpoints.MID;
            case "LOW":
                return (Vars.IS_CONE) ? ConeSetpoints.LOW : CubeSetpoints.LOW;
            case "SUB":
                return (Vars.IS_CONE) ? ConeSetpoints.SUB : CubeSetpoints.SUB;
            case "GRND":
                return (Vars.IS_CONE) ? ConeSetpoints.GRND : CubeSetpoints.GRND;
            default:
                new Alert("Arm", "No setpoint selected", AlertType.ERROR);
                return 0.0;
        }
    }

    /**
     * Arm feedforward required the arm to be at a different idle angle than the
     * robot is usually in. This method corrects for that
     * 
     * @return Adjusted angle for calculating the arm feedforward
     */
    public double getFeedForwardAngle() {
        /* Get the position */
        double angleDegs = m_armInputs.armPosition - Constants.SETPOINTS_FLAT;

        /* Scope adjustment */
        if (angleDegs < 0.0) {
            angleDegs += 360.0;
        }

        // NOTE: Must convert to rads for FF
        return Math.toRadians(angleDegs);
    }
}
