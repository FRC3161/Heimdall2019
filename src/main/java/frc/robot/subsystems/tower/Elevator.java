package frc.robot.subsystems.tower;

import frc.robot.subsystems.tower.Tower.Position;

interface Elevator {
    void setPosition(Position position);
    Position getPosition();
}