package frc.robot.subsystems.tower;

import ca.team3161.lib.robot.LifecycleListener;

public interface Tower extends LifecycleListener {
    enum Position {
        STARTING_CONFIG, //Elevator at bottom and arm in
        GROUND,
        LEVEL_1, // Hatch level 1, cargo level 1, loading
        LEVEL_2,
        LEVEL_3
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
    default void toggleClaw() {
        setClawOpen(!isClawOpen());
    }

    void setBeakOpen(boolean open);
    boolean isBeakOpen();
    default void toggleBeak() {
        setBeakOpen(!isBeakOpen());
    }

    void setRollers(Direction direction);
    Direction getRollerDirection();

    void setElevatorSpeed(double speed);

    void setArmSpeed(double speed);

    void setWristSpeed(double speed);

    void reset();

    void putEncoderTicks();
}
