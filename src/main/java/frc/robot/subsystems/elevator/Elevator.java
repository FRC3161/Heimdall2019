package frc.robot.subsystems.elevator;

public interface Elevator {
    enum Position {
        LOW, // Hatch level 1, cargo level 1, loading
        HATCH_2,
        HATCH_3,
        CARGO_2,
        CARGO_3
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