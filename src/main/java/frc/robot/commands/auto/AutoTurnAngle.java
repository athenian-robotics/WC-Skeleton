package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoTurnAngle extends CommandBase {
    DrivetrainSubsystem drivetrain;
    double tolerance;
    double angleToTurn;
    double setpoint;
    double Kp = 0.016; // CHANGE
    double Ki = 0.0;
    double Kd = 0.001; // CHANGE

    PIDController pid = new PIDController(Kp, Ki, Kd);
    private long startTime;

    public AutoTurnAngle(DrivetrainSubsystem drivetrain, double angleToTurn) {
        this.tolerance = 0.5;
        this.drivetrain = drivetrain;
        this.angleToTurn = angleToTurn;
        pid.setTolerance(tolerance);
        addRequirements(drivetrain);
    }

    public void initialize() {
        setpoint = drivetrain.getGyroAngle() + angleToTurn;
        pid.setSetpoint(setpoint);
        startTime = System.currentTimeMillis();
    }

    public void execute() {

        long elapsedTime = System.currentTimeMillis() - startTime;
        double trapezoidTime = 1000;
        double power;

        if (elapsedTime <= trapezoidTime) {
            power = Math.min(pid.calculate(drivetrain.getGyroAngle()) * drivetrain.calculateTrapezoid(pid,startTime,Constants.maxDriveSpeed,trapezoidTime), Constants.maxDriveSpeed);
        } else {
            power = Math.min(pid.calculate(drivetrain.getGyroAngle()), Constants.maxDriveSpeed); //:
        }

        drivetrain.tankDrive(power, -power);
        System.out.println(power);
        SmartDashboard.putNumber("Angle PID Error:", pid.getPositionError());

    }

    public boolean isFinished() {
        return drivetrain.getGyroAngle() <= setpoint + tolerance && drivetrain.getGyroAngle() >= setpoint - tolerance;
    }

    public void end(boolean interrupted) {
        drivetrain.tankDrive(0, 0);
        System.out.println("Reached setpoint");
    }


}
