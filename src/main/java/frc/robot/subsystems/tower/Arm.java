package frc.robot.subsystems.tower;

import frc.robot.subsystems.tower.Tower.Position;

interface Arm {
    void setPosition(Position position);
    Position getPosition();
    void setEnabled(boolean enabled);
    boolean isEnabled();
}