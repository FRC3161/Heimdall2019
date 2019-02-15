package frc.robot.subsystems.tower;

import ca.team3161.lib.robot.LifecycleListener;
import frc.robot.subsystems.tower.Tower.Position;

interface Arm extends LifecycleListener {
    void setPosition(Position position);
    Position getPosition();
    void setSpeed(double speed);
    void reset();
}