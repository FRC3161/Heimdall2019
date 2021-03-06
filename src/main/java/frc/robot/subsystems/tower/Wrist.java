package frc.robot.subsystems.tower;

import ca.team3161.lib.robot.LifecycleListener;
import frc.robot.subsystems.tower.Tower.Position;

interface Wrist extends LifecycleListener {
    void setPosition(Position position);
    Position getPosition();
    void setSpeed(double speed);
    void reset();
    void task();
    double returnEncoderTicks();
}