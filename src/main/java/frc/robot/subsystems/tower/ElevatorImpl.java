package frc.robot.subsystems.tower;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.subsystems.tower.Tower.Position;

class ElevatorImpl implements Elevator {

    private final TalonSRX controller;

    ElevatorImpl(int talonPort) {
        this.controller = new TalonSRX(talonPort);
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
}