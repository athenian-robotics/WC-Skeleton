package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;


public class AutoDriveForwardTimer extends CommandBase {
    private final DrivetrainSubsystem drivetrainSubsystem;
    Timer t;
    double seconds;

    public AutoDriveForwardTimer(DrivetrainSubsystem drivetrainSubsystem, double secondsToDrive) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.t = new Timer();
        seconds = secondsToDrive;

        addRequirements(this.drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        t.reset();
        t.start();
    }

    @Override
    public void execute() {
        drivetrainSubsystem.arcadeDrive(0.2, 0.2);
    }

    @Override
    public boolean isFinished() {
        return t.hasPeriodPassed(seconds);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.arcadeDrive(0, 0);
        t.stop();
    }
}
