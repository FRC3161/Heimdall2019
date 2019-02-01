package frc.robot.subsystems.elevator;

public interface Elevator {
    enum Position {
    }

    enum Direction {
        NONE,
        IN,
        OUT,
    }

    void setElevatorPosition(Position position);
    Position getElevatorPosition();
    void setClawOpen(boolean open);
    boolean isClawOpen();
    void setRollers(Direction direction);
    Direction getRollerDirection();
    void setEnabled(boolean enabled);
    boolean isEnabled();
}