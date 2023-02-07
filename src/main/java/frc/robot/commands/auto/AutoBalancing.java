package frc.robot.commands.auto;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DrivetrainSubsystem;

import static frc.robot.Constants.maxDriveSpeed;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoBalancing extends CommandBase{
  private final DrivetrainSubsystem drivetrain;
  private final double Kp = 0;
  private final double Ki = 0;
  private final double Kd = 0;

  PIDController pid = new PIDController(Kp, Ki, Kd);

  public AutoBalancing (DrivetrainSubsystem drivetrain) {
    this.drivetrain = drivetrain;
    pid.setTolerance(2.5);
    SmartDashboard.putNumber("p", 0);
  }

  @Override
  public void initialize() {
    pid.setSetpoint(0);
  }

  @Override
  public void execute() {
    double drivePower = Math.min(pid.calculate(drivetrain.getGyroRoll()), maxDriveSpeed);
    drivetrain.tankDrive(-drivePower, drivePower);
    pid.setP(SmartDashboard.getNumber("p", 0));
  }

  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.tankDrive(0, 0);
  }
}
