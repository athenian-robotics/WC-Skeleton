//This Routine just drives forward a set amount
//right now its set to 0.75 meters driven forward
//similar to last years code
package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.AutoForwardDistance;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoRoutine0 extends SequentialCommandGroup {
    public AutoRoutine0(DrivetrainSubsystem drivetrain) {
        addRequirements(drivetrain);
        addCommands(
                new AutoForwardDistance(drivetrain, 0.75)
        );

    }


}