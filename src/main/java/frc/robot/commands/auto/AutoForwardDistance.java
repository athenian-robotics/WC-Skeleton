package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

import static frc.robot.Constants.maxDriveSpeed;

public class AutoForwardDistance extends CommandBase {
    DrivetrainSubsystem drivetrain;
    private final double metersToDrive;
    private final double Kp = 2;
    private final double Ki = 0.0;
    private final double Kd = 0.05;
    private double setpoint = 0.0;
    PIDController pid = new PIDController(Kp, Ki, Kd);

    public AutoForwardDistance(DrivetrainSubsystem drivetrainSubsystem, double metersToDrive) {
        this.metersToDrive = metersToDrive;
        this.drivetrain = drivetrainSubsystem;
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        int sign = setpoint < 0 ? -1 : 1;
        this.setpoint = (drivetrain.getRightEncoderDistance() + drivetrain.getLeftEncoderDistance()) / 2 + metersToDrive + 0.15;
        System.out.println(setpoint);
        pid.setSetpoint(setpoint);
    }

    @Override
    public void execute() {
        double drivePower = Math.min(pid.calculate(drivetrain.getLeftEncoderDistance()), maxDriveSpeed);
        drivePower = Math.abs(drivePower) < 0.4 ? 0 : drivePower;
        drivetrain.tankDrive(-drivePower, -drivePower);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(this.setpoint - (drivetrain.getRightEncoderDistance() + drivetrain.getLeftEncoderDistance()) / 2) < 0.2;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.tankDrive(0, 0);
    }
}