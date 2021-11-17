package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.RobotType;

import static frc.robot.Constants.*;

public class DrivetrainSubsystem extends SubsystemBase {
    SpeedControllerGroup leftMotors;
    SpeedControllerGroup rightMotors;
    private final DifferentialDrive drive;
    public static double maxDriverSpeed = speedScale;

    private final static DrivetrainSubsystem INSTANCE = new DrivetrainSubsystem(RobotType.KITBOT);

    public static DrivetrainSubsystem getInstance() {
        return INSTANCE;
    }

    public DrivetrainSubsystem(RobotType robotType) {

        switch(robotType) {
            case KITBOT:
                leftMotors = new SpeedControllerGroup(new SpeedControllerGroup(new WPI_VictorSPX(leftMotor1Port), new WPI_VictorSPX(leftMotor2Port)));
                rightMotors = new SpeedControllerGroup(new WPI_VictorSPX(rightMotor1Port), new WPI_VictorSPX(rightMotor2Port));
                break;
        }

        leftMotors.setInverted(robotType.isInverted());
        rightMotors.setInverted(robotType.isInverted());

        drive = new DifferentialDrive(leftMotors, rightMotors);
        drive.setDeadband(0.2);
        drive.setMaxOutput(speedScale);
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        int leftSign = leftSpeed >= 0 ? 1 : -1;
        int rightSign = rightSpeed >= 0 ? 1 : -1;

        double leftPower = ((speedScale - minDrivePower) * Math.abs(leftSpeed) + minDrivePower) * leftSign;
        double rightPower = ((speedScale - minDrivePower) * Math.abs(rightSpeed) + minDrivePower) * rightSign;

        SmartDashboard.putNumber("Left power", leftPower);
        SmartDashboard.putNumber("Right power", rightPower);

        drive.tankDrive(leftPower, rightPower);

        //SmartDashboard.putNumber("Left Power:", leftPower);
        //SmartDashboard.putNumber("Right Power:", rightPower);
//        System.out.println(leftPower + " " + rightPower);
    }

    //tank drive for turning autonomous commands
    public void tankDriveTurn(double leftSpeed, double rightSpeed) {
        int leftSign = leftSpeed >= 0 ? 1 : -1;
        int rightSign = rightSpeed >= 0 ? 1 : -1;

        double leftPower = ((speedScale - minDrivePowerTurn) * Math.abs(leftSpeed) + minDrivePowerTurn) * leftSign;
        double rightPower = ((speedScale - minDrivePowerTurn) * Math.abs(rightSpeed) + minDrivePowerTurn) * rightSign;

        drive.tankDrive(leftPower, rightPower);
    }

    public void setMaxOutput(double maxOutput) {
        drive.setMaxOutput(maxOutput);
    }

    public void arcadeDrive(double xSpeed, double zRotation) {
        //drive.arcadeDrive(xSpeed*maxDriverSpeed, -zRotation*maxDriverSpeed);

        //account for changes in turning when the forward direction changes, if it doesn't work use the one above
        drive.arcadeDrive(xSpeed * maxDriverSpeed, maxDriverSpeed < 0 ? zRotation * maxDriverSpeed : -zRotation * maxDriverSpeed);
    }

    @Override
    public void periodic() {

    }
}

