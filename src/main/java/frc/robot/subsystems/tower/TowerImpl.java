package frc.robot.subsystems.tower;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.subsystems.tower.Elevator;

import org.apache.commons.collections4.BidiMap;
import org.apache.commons.collections4.bidimap.DualHashBidiMap;
import org.apache.commons.collections4.bidimap.UnmodifiableBidiMap;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.VictorSP;

public class TowerImpl implements Tower {

    private static final BidiMap<Direction, Double> ROLLER_DIRECTIONS_PWM;
    static {
        final BidiMap<Direction, Double> rollerPwms = new DualHashBidiMap<>();
        // Direction to Roller PWM mapping
        rollerPwms.put(Direction.NONE, Double.valueOf(0));
        rollerPwms.put(Direction.IN, Double.valueOf(1));
        rollerPwms.put(Direction.OUT, Double.valueOf(-1));
        ROLLER_DIRECTIONS_PWM = UnmodifiableBidiMap.unmodifiableBidiMap(rollerPwms);
    }

    private final Elevator elevator;
    private final Arm arm;
    private Position position;
    private Solenoid claw;
    private SpeedControllerGroup roller;

    public TowerImpl() {
        this.elevator = new ElevatorImpl(8);
        this.arm = new ArmImpl(7);
        this.claw = new Solenoid(RobotMap.CLAW_SOLENOID); //Placeholder port
        this.roller = new SpeedControllerGroup(new VictorSP(RobotMap.TOWER_ROLLER_1), new VictorSP(RobotMap.TOWER_ROLLER_2));
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
        if (!ROLLER_DIRECTIONS_PWM.containsKey(direction)) {
            roller.set(0);
            return;
        }
        roller.set(ROLLER_DIRECTIONS_PWM.get(direction));
    }

    @Override
    public Direction getRollerDirection() {
        double rollerSpeed = roller.get();
        if (!ROLLER_DIRECTIONS_PWM.inverseBidiMap().containsKey(rollerSpeed)) {
            return Direction.NONE;
        }
        return ROLLER_DIRECTIONS_PWM.inverseBidiMap().get(rollerSpeed);
    }

    public void setElevatorSpeed(double speed) {
        this.elevator.setSpeed(speed);
    }

    public void setArmSpeed(double speed){
        this.arm.setSpeed(speed);
    }
}
