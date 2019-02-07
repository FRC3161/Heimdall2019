package frc.robot.subsystems.tower;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.tower.Elevator;
import edu.wpi.first.wpilibj.Solenoid;

public class TowerImpl implements Tower {

    private final Elevator elevator;
    private final Arm arm;
    private Position position;
    private Solenoid solenoid;

    public TowerImpl() {
        this.elevator = new ElevatorImpl(8);
        this.arm = new ArmImpl(7);
        this.solenoid = solenoid;
        setTowerPosition(Position.STARTING_CONFIG);
    }

    @Override
    public void setTowerPosition(Position position) {
        this.position = position;
        this.elevator.setPosition(position);
        this.arm.setPosition(position);
        SmartDashboard.putString("Elevator Position", position.toString());
    }

    @Override
    public Position getTowerPosition() {
        return position;
    }

    @Override
    public void setClawOpen(boolean open) {
        solenoid.set(open);
        //TODO might need a SpeedController? (see setDeployed in ColsonPodImpl)
    }

    @Override
    public boolean isClawOpen() {
        //TODO
        return false;
    }

    @Override
    public void setBeakOpen(boolean open) {
        //TODO
    }

    @Override
    public boolean isBeakOpen() {
        //TODO
        return false;
    }

    @Override
    public void setRollers(Direction direction) {
        //TODO
    }

    @Override
    public Direction getRollerDirection() {
        //TODO
        return null;
    }

    public void setElevatorSpeed(double speed) {
        this.elevator.setSpeed(speed);
    }
    
    public void setArmSpeed(double speed){
        this.arm.setSpeed(speed);
    }
}