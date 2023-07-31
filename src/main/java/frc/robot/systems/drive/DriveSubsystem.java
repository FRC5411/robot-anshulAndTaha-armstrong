package frc.robot.systems.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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

  public Command arcadeCmd(DoubleSupplier speedSupplier, DoubleSupplier rotationSupplier, BooleanSupplier sniperMode) {
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

  public Command resetOdometryCmd() {
    return resetOdometryCmd(new Pose2d());
  }

  public Command resetOdometryCmd(Pose2d pose) {
    return new InstantCommand(() -> IO.resetPose(pose), this);
  }

  public Command autoEngageCmd() {
    ProfiledPIDController engageController = new ProfiledPIDController(
      0.0243, //0.0234, //0.027
      0,
      0,
      new TrapezoidProfile.Constraints(1, 0.5));

      engageController.setTolerance(2);

    double kf = 0.05;

    return new FunctionalCommand(
      () -> {}, 
      () -> {
        IO.arcadeDrive(
          engageController.calculate(
            IO.getPitch(), 0) + kf * Math.signum(engageController.getPositionError()), 0);}, 
      interrupted -> {}, 
      () -> false, 
      this);
  }

  public Command turnCommand(double setpoint) {
    ProfiledPIDController controller = new ProfiledPIDController(
      0.03, 
      0, 
      0.005, 
      new TrapezoidProfile.Constraints(180, 90));

      double kf = 0.05;

    return new FunctionalCommand(
      () -> { controller.reset(IO.getHeading().getDegrees());}, 
      () -> {IO.arcadeDrive(
        controller.calculate(IO.getHeading().getDegrees(), setpoint) 
        + kf * Math.signum(controller.getPositionError()), 0.0);}, 
      interrupted -> {}, 
      () -> false, 
      this);
  }
}