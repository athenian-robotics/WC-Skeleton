package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoForwardDistance extends CommandBase {
    private final DrivetrainSubsystem drivetrainSubsystem;
    private final Timer driveTimer;
    private final PIDController pid;
    private final double trapezoidTime;
    private final double metersToDrive;
    private long startTime;
    private double encoderDifferenceSetpoint;

    public AutoForwardDistance(DrivetrainSubsystem drivetrainSubsystem, double metersToDrive) {
        this(drivetrainSubsystem, metersToDrive, 1000); // Default to trapezoid time 1000
    }

    public AutoForwardDistance(DrivetrainSubsystem drivetrainSubsystem, double metersToDrive, double trapezoidTime) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        double tolerance = 0.01; // Define an acceptable error for PID
        this.metersToDrive = metersToDrive + tolerance; // Define how far we should drive (in meters)
        this.trapezoidTime = trapezoidTime;

        driveTimer = new Timer();
        pid = new PIDController(1, 0, 0.02); // Set up the drive timer and PID controller
        pid.setTolerance(tolerance);

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
        driveTimer.reset();
        driveTimer.start(); // Start the drive timer

        double setpoint = drivetrainSubsystem.getRightEncoderDistance() + metersToDrive; // Current position + desired position
        encoderDifferenceSetpoint = drivetrainSubsystem.getLeftEncoderDistance() - drivetrainSubsystem.getRightEncoderDistance(); // Difference in encoders
        pid.setSetpoint(setpoint); // Set the pid controller's setpoint
    }

    @Override
    public void execute() {
        double power = drivetrainSubsystem.calculateTrapezoid(pid, startTime, Constants.maxDriveSpeed, this.trapezoidTime); // Calculate trapezoid speed
        System.out.println("== POWER : " + power);

        double leftCorrection = drivetrainSubsystem.leftEncoderCorrection(encoderDifferenceSetpoint); // Calculate correction needed for left encoder
        double rightCorrection = drivetrainSubsystem.rightEncoderCorrection(encoderDifferenceSetpoint); // Calculate correction needed for right encoder

        drivetrainSubsystem.tankDrive(power + leftCorrection, power + rightCorrection); // Drive accounting for correction values

    }

    @Override
    public boolean isFinished() {
        System.out.println("== Finished ==");
        return pid.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("== Reached setpoint ==");
        drivetrainSubsystem.tankDrive(0, 0);
    }
}