package frc.robot.subsystems.tower;

import ca.team3161.lib.robot.LifecycleListener;

public interface Tower extends LifecycleListener {
    enum Position {
        STARTING_CONFIG, //Elevator at bottom and arm in
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

    void setTowerPosition(Position position);
    Position getTowerPosition();
    void setClawOpen(boolean open);
    boolean isClawOpen();
    void setBeakOpen(boolean open);
    boolean isBeakOpen();
    void setRollers(Direction direction);
    Direction getRollerDirection();
    void setElevatorSpeed(double speed);
    void setArmSpeed(double speed);
}