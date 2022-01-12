package frc.robot.lib;

public enum RobotType {
    KITBOT(false), JANKBOT(false), OFFICIAL(false); // This ENUM defines all types of robots that utilize west-coast drivetrain

    private final boolean inverted; // Each robot will have a variable determining its inversion

    RobotType(boolean inverted) {
        this.inverted = inverted;
    } // Initialize the inverted instance variable

    public boolean isInverted() {
        return this.inverted;
    } // Typical getter
}
