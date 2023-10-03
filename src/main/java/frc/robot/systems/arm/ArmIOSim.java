// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.arm;

import edu.wpi.first.math.MathUtil;
import frc.robot.systems.arm.ArmVars.Simulation;

/** Simulates a single jointed arm */
public class ArmIOSim implements ArmIO {

    /** Creates a new simulation for the arm */
    public ArmIOSim() {}

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.armPosition = Simulation.SIM_ARM_ENCODER.getDistance();
        inputs.armVelocity = Simulation.SIM_ARM_ENCODER.getRate();
        inputs.armTemperatureC = 0.0;
        inputs.armAppliedCurrent = Simulation.SIM_ARM.getCurrentDrawAmps();
    }

    public void setVolts(double volts) {
        volts = MathUtil.clamp(volts, -12.0, 12.0);
        Simulation.SIM_ARM.setInput(volts);
    }
}
