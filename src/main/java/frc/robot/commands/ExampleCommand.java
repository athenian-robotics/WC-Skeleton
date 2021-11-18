package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ExampleCommand extends CommandBase {
    /**
     * This example command exists outside the drive package so that
     * the drive package does not combine with the commands package. Once
     * actual commands exist outside the drive package, this file will
     * not be necessary.
     */

    @Override
    public void initialize() {
        // Runs once when the command is scheduled
    }

    @Override
    public void execute() {
        // Executed continuously when the command is scheduled
    }

    @Override
    public boolean isFinished() {
        return false; // Replace with an actual condition
    }

    @Override
    public void end(boolean interrupted) {
        // Runs once it is time to terminate execution
    }
}
