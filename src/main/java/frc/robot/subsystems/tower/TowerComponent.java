package frc.robot.subsystems.tower;

import ca.team3161.lib.robot.LifecycleListener;

interface TowerComponent extends LifecycleListener {
    boolean atPosition();
}