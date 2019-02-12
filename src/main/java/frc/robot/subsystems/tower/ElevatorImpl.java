package frc.robot.subsystems.tower;

import static frc.robot.MathUtils.absClamp;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.subsystems.tower.Tower.Position;

class ElevatorImpl implements Elevator {

    private final WPI_TalonSRX controllerMaster;
    private final WPI_TalonSRX controllerSlave;

    ElevatorImpl(int masterPort, int slavePort) {
        this.controllerMaster = new WPI_TalonSRX(masterPort);
        this.controllerSlave = new WPI_TalonSRX(slavePort);
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
        controllerMaster.set(absClamp(speed, maxPower));
        controllerSlave.set(absClamp(speed, maxPower));
    }

}