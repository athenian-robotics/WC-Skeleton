package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber;

// See http://docs.limelightvision.io/en/latest/networktables_api.html

public class LimeLightSubsystem extends SubsystemBase {

    final NetworkTable limelight;


    public LimeLightSubsystem(String tableName) {
        this.limelight = NetworkTableInstance.getDefault().getTable(tableName);
    }

    public void periodic() {
        double x = limelight.getEntry("tlong").getDouble(0.0);

        putNumber("tlong", x);
        putNumber("distance in cm", 40/Math.tan(59.6*x/(2*960)));
    }

    public NetworkTable grabNetworkTable() {
        return this.limelight;
    }
}
