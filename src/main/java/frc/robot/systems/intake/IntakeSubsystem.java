// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.systems.intake.IntakeVars.Constants;
import frc.robot.systems.intake.IntakeVars.Objects;
import frc.robot.systems.intake.IntakeVars.Vars;

/** Intake subsystem class */
public class IntakeSubsystem extends SubsystemBase {

    private final IntakeIO m_intakeIO;

    private final IntakeIOInputsAutoLogged m_intakeInputs = new IntakeIOInputsAutoLogged();

    /**
     * Creates a new instance of the intake subsystem
     * 
     * @param intakeIO Type of IO the subsystem will be using
     */
    public IntakeSubsystem(IntakeIO intakeIO) {
        m_intakeIO = intakeIO;
    }

    @Override
    public void periodic() {
        /* Update inputs */
        m_intakeIO.updateInputs(m_intakeInputs);
        Logger.getInstance().processInputs("/systems/intake/intakeIO", m_intakeInputs);

        /* Calculate intake motor output average (25 samples) */
        double calc = Objects.INTAKE_LIN_FILTER.calculate(m_intakeInputs.intakeOutputCurrent);

        /* Check if the average output current is more than our current limits */
        if (calc > Constants.INTAKE_CURRENT_LIMIT) {
            /* If so, stop the motor */
            setSpeeds(0.0);
        }
    }

    /**
     * Sets the volts of the motor (-12 to 12)
     * 
     * @param volts Volts
     */
    public void setSpeeds(double volts) {
        m_intakeIO.setVolts(volts);
    }

    /**
     * Intakes a gamepiece
     * 
     * @param isCone If the gamepiece is a cone or cube (not required)
     */
    public void intake(Boolean isCone) {
        if (isCone.equals(null)) {
            if (Vars.IS_CONE) {
                setSpeeds(-(Constants.PERCENT_CONE_SPEED * 12.0));
            } else {
                setSpeeds((Constants.PERCENT_CUBE_SPEED * 12.0));
            }
        } else {
            if (isCone) {
                setSpeeds(-(Constants.PERCENT_CONE_SPEED * 12.0));
            } else {
                setSpeeds((Constants.PERCENT_CUBE_SPEED * 12.0));
            }
        }
    }

    /**
     * Outtakes a gamepiece
     * 
     * @param isCone If the gamepiece is a cone or cube (not required)
     */
    public void outtake(Boolean isCone) {
        if (isCone.equals(null)) {
            if (Vars.IS_CONE) {
                setSpeeds((Constants.PERCENT_CONE_SPEED * 12.0));
            } else {
                setSpeeds(-(Constants.PERCENT_CUBE_SPEED * 12.0));
            }
        } else {
            if (isCone) {
                setSpeeds((Constants.PERCENT_CONE_SPEED * 12.0));
            } else {
                setSpeeds(-(Constants.PERCENT_CUBE_SPEED * 12.0));
            }
        }
    }
}
