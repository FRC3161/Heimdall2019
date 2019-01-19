package frc.robot.subsystems.elevator;

public interface Elevator {
    enum Position {
    }

    void setPosition(Position position);
    Position getPosition();
    void setClawOpen(boolean open);
    void isClawOpen();
    void setEnabled(boolean enabled);
    boolean isEnabled();
}