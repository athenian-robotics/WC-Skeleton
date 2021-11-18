package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.RobotType;

import static frc.robot.Constants.*;

public class DrivetrainSubsystem extends SubsystemBase {
    SpeedControllerGroup leftMotors; // With more than one motor on each side, we use SpeedControllerGroup
    SpeedControllerGroup rightMotors;
    private final DifferentialDrive drive; // DifferentialDrive manages steering based off of inputted power values
    public static double maxDriverSpeed = speedScale;

    private final static DrivetrainSubsystem INSTANCE = new DrivetrainSubsystem(RobotType.KITBOT); // SubsystemBase native

    public static DrivetrainSubsystem getInstance() {
        return INSTANCE;
    }

    public DrivetrainSubsystem(RobotType robotType) {
        // Ports are defined in Constants
        switch(robotType) {
            // Kitbot utilizes TalonSRXs as the motor controllers.
            case JANKBOT:
                leftMotors = new SpeedControllerGroup(new WPI_VictorSPX(leftMotor1Port), new WPI_VictorSPX(leftMotor2Port));
                rightMotors = new SpeedControllerGroup(new WPI_VictorSPX(rightMotor1Port), new WPI_VictorSPX(rightMotor2Port));
                break;
            case KITBOT:
                leftMotors = new SpeedControllerGroup(new SpeedControllerGroup(new WPI_TalonSRX(leftMotor1Port), new WPI_TalonSRX(leftMotor2Port)));
                rightMotors = new SpeedControllerGroup(new WPI_TalonSRX(rightMotor1Port), new WPI_TalonSRX(rightMotor2Port));
                break;
        }

        leftMotors.setInverted(robotType.isInverted());
        rightMotors.setInverted(robotType.isInverted());

        drive = new DifferentialDrive(leftMotors, rightMotors);
        drive.setDeadband(0.2); // Scales joystick values. 0.02 is the default
        drive.setMaxOutput(speedScale);

    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        int leftSign = leftSpeed >= 0 ? 1 : -1; // Checks leftSpeed and gathers whether it is negative or positive
        int rightSign = rightSpeed >= 0 ? 1 : -1; // Checks rightSpeed and gathers whether it is negative or positive

        double leftPower = ((speedScale - minDrivePower) * Math.abs(leftSpeed) + minDrivePower) * leftSign;
        double rightPower = ((speedScale - minDrivePower) * Math.abs(rightSpeed) + minDrivePower) * rightSign;

        drive.tankDrive(leftPower, rightPower); // Calls WPILib DifferentialDrive method tankDrive(LSpeed, RSpeed)
    }

    public void setMaxOutput(double maxOutput) {
        drive.setMaxOutput(maxOutput);
    }

    public void arcadeDrive(double xSpeed, double zRotation) {
        // Account for changes in turning when the forward direction changes, if it doesn't work use the one above
        drive.arcadeDrive(xSpeed * maxDriverSpeed, maxDriverSpeed < 0 ? zRotation * maxDriverSpeed : -zRotation * maxDriverSpeed);
    }

    @Override
    public void periodic() {
        // SubsystemBase native method
    }
}

