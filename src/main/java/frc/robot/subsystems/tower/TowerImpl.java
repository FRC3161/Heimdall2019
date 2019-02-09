package frc.robot.subsystems.tower;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.tower.Elevator;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.VictorSP;

public class TowerImpl implements Tower {

    private final Elevator elevator;
    private final Arm arm;
    private Position position;
    private Solenoid claw;
    private SpeedControllerGroup roller;

    public TowerImpl() {
        this.elevator = new ElevatorImpl(8);
        this.arm = new ArmImpl(7);
        this.claw = new Solenoid(1); //Placeholder port
        this.roller = new SpeedControllerGroup(new VictorSP(9), new VictorSP(10));  //TODO Placeholder ports
        setTowerPosition(Position.STARTING_CONFIG);
    }

    @Override
    public void setTowerPosition(Position position) {
        this.position = position;
        this.elevator.setPosition(position);
        this.arm.setPosition(position);
        SmartDashboard.putString("Tower Position", position.toString());
    }

    @Override
    public Position getTowerPosition() {
        return position;
    }

    @Override
    public void setClawOpen(boolean open) {
        claw.set(open);
        //TODO might need a SpeedController? (see setDeployed in ColsonPodImpl)
    }

    @Override
    public boolean isClawOpen() {
        return claw.get();
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
        if (direction.equals(Direction.IN)) {
            roller.set(1.0);
          } else if (direction.equals(Direction.OUT)) {
            roller.set(-1.0);
          } else {
            roller.set(0.0);
          }
    }

    @Override
    public Direction getRollerDirection() {
        double rollerSpeed = roller.get();
        if (rollerSpeed > 0) {
          return Direction.IN;
        }
        else if (rollerSpeed < 0) {
            return Direction.OUT;
        }
        else {
            return Direction.NONE;
        }
    }

    public void setElevatorSpeed(double speed) {
        this.elevator.setSpeed(speed);
    }
    
    public void setArmSpeed(double speed){
        this.arm.setSpeed(speed);
    }
}
