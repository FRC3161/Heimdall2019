package frc.robot.subsystems.tower;

import frc.robot.subsystems.tower.Tower.Position;

interface Arm extends TowerComponent {
    void setPosition(Position position);
    Position getPosition();
    void setSpeed(double speed);
    void reset();
    void task();
    double returnEncoderTicks();
}