package frc.robot.subsystems.tower;

import static frc.robot.MathUtils.absClamp;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.tower.Tower.Position;

class ElevatorImpl implements Elevator {

    private final WPI_TalonSRX controllerMaster;
    private final WPI_TalonSRX controllerSlave;
    private final DigitalInput limitSwitchTop;
    private final DigitalInput limitSwitchBottom;

    ElevatorImpl(int masterPort, int slavePort, int topSwitchPort , int bottomSwitchPort) {
        this.controllerMaster = new WPI_TalonSRX(masterPort);
        this.controllerSlave = new WPI_TalonSRX(slavePort);
        this.limitSwitchTop = new DigitalInput(topSwitchPort);
        this.limitSwitchBottom = new DigitalInput(bottomSwitchPort);
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
        while(limitSwitchTop.get() == true){
            maxPower = 0;
        }
        while (limitSwitchBottom.get() == true) {
            if (speed < 0){
                speed = 0; 
            }
        }
        controllerMaster.set(absClamp(speed, maxPower));
        controllerSlave.set(absClamp(speed, maxPower));
    }

}