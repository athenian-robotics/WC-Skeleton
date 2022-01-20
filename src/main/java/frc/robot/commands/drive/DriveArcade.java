package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveArcade extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DrivetrainSubsystem drivetrainSubsystem;
    private final XboxController xboxController;


    public DriveArcade(DrivetrainSubsystem drivetrainSubsystem, XboxController xboxController) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.xboxController = xboxController;

        addRequirements(drivetrainSubsystem); // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //See WPILib for more extensive instructions
        drivetrainSubsystem.arcadeDrive(-xboxController.getLeftY(),
                -xboxController.getRightX());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}