package frc.robot.lib;

public enum RobotType {
    KITBOT(false), JANKBOT(false);

    private final boolean inverted;

    RobotType(boolean inverted) {
        this.inverted = inverted;
    }

    public boolean isInverted() {
        return this.inverted;
    }
}
