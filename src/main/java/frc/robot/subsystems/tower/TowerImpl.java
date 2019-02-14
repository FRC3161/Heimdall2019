package frc.robot.subsystems.tower;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.subsystems.tower.Elevator;

import org.apache.commons.collections4.BidiMap;
import org.apache.commons.collections4.bidimap.DualHashBidiMap;
import org.apache.commons.collections4.bidimap.UnmodifiableBidiMap;

import ca.team3161.lib.robot.LifecycleEvent;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.VictorSP;

public class TowerImpl implements Tower {

    private static final BidiMap<Direction, Double> ROLLER_DIRECTIONS_PWM;
    static {
        final BidiMap<Direction, Double> rollerPwms = new DualHashBidiMap<>();
        // Direction to Roller PWM mapping
        rollerPwms.put(Direction.NONE, 0.);
        rollerPwms.put(Direction.IN, 1.);
        rollerPwms.put(Direction.OUT, -1.);
        ROLLER_DIRECTIONS_PWM = UnmodifiableBidiMap.unmodifiableBidiMap(rollerPwms);
    }

    private final Elevator elevator;
    private final Arm arm;
    private final Solenoid claw;
    private final Solenoid beak;
    private final SpeedControllerGroup roller;
    private final GamePieceWatcher gamePieceWatcher;
    private Position position;

    public TowerImpl() {
        this.elevator = new ElevatorImpl(RobotMap.ELEVATOR_MASTER_CONTROLLER, RobotMap.ELEVATOR_SLAVE_CONTROLLER, RobotMap.TOP_LIMIT_SWITCH,RobotMap.BOTTOM_LIMIT_SWITCH);
        this.arm = new ArmImpl(RobotMap.ARM_CONTROLLER);
        this.claw = new Solenoid(RobotMap.CLAW_SOLENOID);
        this.beak = new Solenoid(RobotMap.BEAK_SOLENOID);
        this.roller = new SpeedControllerGroup(new VictorSP(RobotMap.TOWER_ROLLER_1), new VictorSP(RobotMap.TOWER_ROLLER_2));
        this.gamePieceWatcher = new GamePieceWatcher();
        this.gamePieceWatcher.start();
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
        beak.set(open);
    }

    @Override
    public boolean isBeakOpen() {
        return beak.get();
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

    @Override
    public void lifecycleStatusChanged(LifecycleEvent previous, LifecycleEvent current) {
        this.gamePieceWatcher.lifecycleStatusChanged(previous, current);
    }
    @Override
    public void reset(){
        this.elevator.reset();
    }
}
