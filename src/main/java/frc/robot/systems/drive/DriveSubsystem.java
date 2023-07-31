package frc.robot.systems.drive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class DriveSubsystem extends SubsystemBase {
  DriveIO IO;
  DriveIOSim simIO;
  

  public DriveSubsystem() {
    IO = new DriveIO();
    simIO = new DriveIOSim();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  @Override
  public void periodic() {
    IO.updateOdometry();
    IO.telemetry();
    IO.setField();
  }

  @Override
  public void simulationPeriodic() {
    simIO.update();
    simIO.telemetry();
    simIO.setField();
  }

  public Command arcadeCommand(DoubleSupplier speedSupplier, DoubleSupplier rotationSupplier, BooleanSupplier sniperMode) {
    return new FunctionalCommand(
      () -> {},
      () -> {IO.teleopDrive(
        speedSupplier.getAsDouble(), 
        rotationSupplier.getAsDouble(),
        sniperMode.getAsBoolean());},
      interrupted -> {},
      () -> false,
      this);
  }

  public Command sniperTrueCmd() {
    return new InstantCommand(() -> IO.trueSniperMode(), this);
  }

  public Command sniperFalseCmd() {
    return new InstantCommand(() -> IO.falseSniperMode(), this);
  }


}