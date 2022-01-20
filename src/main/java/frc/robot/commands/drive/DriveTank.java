package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;


public class DriveTank extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DrivetrainSubsystem drivetrainSubsystem;
    private final XboxController xboxController;

    public DriveTank(DrivetrainSubsystem drivetrainSubsystem, XboxController xboxController) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.xboxController = xboxController;
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        //See WPILib for more extensive instructions
        drivetrainSubsystem.tankDrive(-xboxController.getLeftY(),
                -xboxController.getRightX());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
