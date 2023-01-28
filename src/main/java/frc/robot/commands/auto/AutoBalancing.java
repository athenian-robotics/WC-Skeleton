package frc.robot.commands.auto;

import frc.robot.subsystems.DrivetrainSubsystem;

import static frc.robot.Constants.maxDriveSpeed;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoBalancing extends CommandBase{
  private final DrivetrainSubsystem drivetrain;
  private final double Kp = 0.5;
  private final double Ki = 0;
  private final double Kd = 0.5;
  private final double setpoint = 0.0;
  PIDController pid = new PIDController(Kp, Ki, Kd);

  public AutoBalancing (DrivetrainSubsystem drivetrain) {
    this.drivetrain = drivetrain;
  }

  @Override
  public void initialize() {
    pid.setSetpoint(setpoint);
  }

  @Override
  public void execute() {
    double drivePower = Math.min(pid.calculate(drivetrain.getGyroPitch()), maxDriveSpeed);
    drivePower = Math.abs(drivePower) < 0.4 ? 0 : drivePower;
    drivetrain.tankDrive(-drivePower, -drivePower);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(drivetrain.getGyroPitch()) < 0.1;
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.tankDrive(0, 0);
  }
}
