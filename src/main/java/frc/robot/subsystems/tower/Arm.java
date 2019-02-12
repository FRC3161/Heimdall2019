package frc.robot.subsystems.tower;

import frc.robot.subsystems.tower.Tower.Position;

interface Arm {
    void setPosition(Position position);
    Position getPosition();
    void setSpeed(double speed);
    void pidLoop();
}