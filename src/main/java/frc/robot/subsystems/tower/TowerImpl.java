package frc.robot.subsystems.tower;
import frc.robot.subsystems.tower.Elevator;
public class TowerImpl implements Tower {

    private final Elevator elevator;
    private final Arm arm;
    private Position position;

    public TowerImpl() {
        this.elevator = new ElevatorImpl(8);
        this.arm = new ArmImpl(7);
        this.position = Position.LOW;
    }

    @Override
    public void setTowerPosition(Position position) {
        this.position = position;
        this.elevator.setPosition(position);
        this.arm.setPosition(position);
    }

    @Override
    public Position getTowerPosition() {
        return position;
    }

    @Override
    public void setClawOpen(boolean open) {
        //TODO
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