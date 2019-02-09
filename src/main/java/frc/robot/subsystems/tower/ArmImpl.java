package frc.robot.subsystems.tower;

import static frc.robot.MathUtils.absClamp;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.subsystems.tower.Tower.Position;

class ArmImpl implements Arm {

    private final WPI_TalonSRX controller;

    ArmImpl(int talonPort) {
        this.controller = new WPI_TalonSRX(talonPort);
    }

    @Override
    public void setPosition(Position position) {
        //TODO
    }

    @Override
    public Position getPosition() {
        //TODO
        return null;
    }

    @Override
    public void setSpeed(double speed) {
        double maxPower= 0.25;
        this.controller.set(absClamp(speed, maxPower));
    }
}