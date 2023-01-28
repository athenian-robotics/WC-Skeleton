package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.auto.GoalNotFoundException;

// See http://docs.limelightvision.io/en/latest/networktables_api.html


public class LimeLightSubsystem extends SubsystemBase {
    final NetworkTable limelight;
    double[] limelightOutputArray;
    double[] defaultLimelightOutputArray = {-1, -1, -1, -1, -1, -1, -1, -1};

    public LimeLightSubsystem(String tableName) {
        this.limelight = NetworkTableInstance.getDefault().getTable(tableName);
        limelightOutputArray = defaultLimelightOutputArray;
    }

    public double getLimelightOutputAtIndex(int index) throws GoalNotFoundException {
        if (index>8||index<0) {
            throw new IndexOutOfBoundsException();
        } else if (limelightOutputArray!=defaultLimelightOutputArray) {
            return limelightOutputArray[index];
        } else {
            throw new GoalNotFoundException();
        }
    }

    public void periodic() {
        limelightOutputArray = limelight.getEntry("llpython").getDoubleArray(defaultLimelightOutputArray);
        System.out.println(limelightOutputArray[1]);
        SmartDashboard.putNumber("xOffset", limelightOutputArray[1]);
    }
}

