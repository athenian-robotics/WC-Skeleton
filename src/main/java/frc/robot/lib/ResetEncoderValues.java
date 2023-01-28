package frc.robot.lib;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;


public class ResetEncoderValues extends CommandBase {
    private final DrivetrainSubsystem drivetrainSubsystem;

    public ResetEncoderValues(DrivetrainSubsystem drivetrainSubsystem) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        addRequirements(this.drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        drivetrainSubsystem.resetRightEncoder();
        drivetrainSubsystem.resetLeftEncoder();
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
