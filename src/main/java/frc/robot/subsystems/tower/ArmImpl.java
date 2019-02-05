package frc.robot.subsystems.tower;

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
    public void setSpeed(double speed){
        double output;
        double maxPower= 0.25;
    
        if (speed > 0) {
            if (speed >= maxPower) {
                output = maxPower;
            } else {
                 output = speed;
            }
        } else {
            if (Math.abs(speed) >= maxPower) {
                 output = -maxPower;
            } else {
                output = speed;
            }
        }
        this.controller.set(output);
    }
}