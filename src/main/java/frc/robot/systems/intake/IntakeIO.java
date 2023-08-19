
package frc.robot.systems.intake;

import frc.robot.systems.intake.IntakeVars.Objects;

/** Add your docs here. */
public class IntakeIO {
    public IntakeIO() {}

    public void setIntake(double speed) {
        Objects.intakeMotor.set(speed);
    }

    public double getIntakeCurrent() {
        return Objects.intakeMotor.getOutputCurrent();
    }
}
