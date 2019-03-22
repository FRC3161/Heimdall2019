package frc.robot.subsystems.tower;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.subsystems.tower.Elevator;

import org.apache.commons.collections4.BidiMap;
import org.apache.commons.collections4.bidimap.DualHashBidiMap;
import org.apache.commons.collections4.bidimap.UnmodifiableBidiMap;

import ca.team3161.lib.robot.LifecycleEvent;
import ca.team3161.lib.utils.Utils;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.DigitalInput;

public class TowerImpl implements Tower {

    private static final BidiMap<Direction, Double> ROLLER_DIRECTIONS_PWM;
    static {
        final BidiMap<Direction, Double> rollerPwms = new DualHashBidiMap<>();
        // Direction to Roller PWM mapping
        rollerPwms.put(Direction.NONE, 0.);
        rollerPwms.put(Direction.IN, 0.7);
        rollerPwms.put(Direction.OUT, -1.);
        ROLLER_DIRECTIONS_PWM = UnmodifiableBidiMap.unmodifiableBidiMap(rollerPwms);
    }

    private final Elevator elevator;
    private final Arm arm;
    private final Solenoid openBeak;
    private final Solenoid closeBeak;
    private final SpeedController intake;
    private final SpeedController wrist;
    private final GamePieceWatcher gamePieceWatcher;
    private final DigitalInput limitSwitchWrist;
    private Position position;

    public TowerImpl() {
        this.elevator = Utils.safeInit("elevator", () -> new ManualElevatorImpl(RobotMap.ELEVATOR_MASTER_CONTROLLER, RobotMap.ELEVATOR_SLAVE_CONTROLLER, RobotMap.TOP_LIMIT_SWITCH,RobotMap.BOTTOM_LIMIT_SWITCH));
        this.arm = Utils.safeInit("arm", () -> new WpiPidArmImpl(RobotMap.ARM_CONTROLLER));
        this.openBeak = Utils.safeInit("openBeak", () -> new Solenoid(RobotMap.BEAK_OPEN_SOLENOID));
        this.closeBeak = Utils.safeInit("closeBeak", () -> new Solenoid(RobotMap.BEAK_CLOSE_SOLENOID));
        this.intake = Utils.safeInit("intake", () -> new VictorSP(RobotMap.TOWER_ROLLER_INTAKE));
        this.wrist = Utils.safeInit("wrist", () -> new  VictorSP(RobotMap.TOWER_ROLLER_WRIST));
        this.limitSwitchWrist = Utils.safeInit("limitSwitchWrist", () -> new DigitalInput(RobotMap.WRIST_LIMIT_SWITCH));
        this.gamePieceWatcher = Utils.safeInit("gamePieceWatcher", () -> new GamePieceWatcher());
        setTowerPosition(Position.STARTING_CONFIG);
        wrist.setInverted(false);
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
    public void setBeakOpen(boolean open) {
        openBeak.set(open);
        closeBeak.set(!open);
    }

    @Override
    public boolean isBeakOpen() {
        return  openBeak.get();
    }

    @Override
    public void setRollers(Direction direction) {
        if (!ROLLER_DIRECTIONS_PWM.containsKey(direction)) {
            intake.set(0);
            return;
        }
        intake.set(ROLLER_DIRECTIONS_PWM.get(direction));
    }

    @Override
    public Direction getRollerDirection() {
        double rollerSpeed = intake.get();
        if (!ROLLER_DIRECTIONS_PWM.inverseBidiMap().containsKey(rollerSpeed)) {
            return Direction.NONE;
        }
        return ROLLER_DIRECTIONS_PWM.inverseBidiMap().get(rollerSpeed);
    }

    @Override
    public void setElevatorSpeed(double speed) {
        this.elevator.setSpeed(speed);
    }

    @Override
    public void setArmSpeed(double speed){
        this.arm.setSpeed(speed);
    }

    @Override
    public void setWristSpeed(double speed){
        if (speed == 0){
            speed = 0.08;
        }
        // if (this.arm.returnEncoderTicks() >= 0){
        //    if (speed < 0){
        //     this.wrist.set(0);
        //     return;
        //    }
        // }
        this.wrist.set(speed / -2);
    }

    @Override
    public void lifecycleStatusChanged(LifecycleEvent previous, LifecycleEvent current) {
        if (current.equals(LifecycleEvent.ON_INIT)) {
            this.gamePieceWatcher.start();
        }
        this.gamePieceWatcher.lifecycleStatusChanged(previous, current);
        this.elevator.lifecycleStatusChanged(previous, current);
        this.arm.lifecycleStatusChanged(previous, current);
    }

    @Override
    public void reset(){
        this.elevator.reset();
        this.arm.reset();
    }

    @Override
    public void putEncoderTicks() {
        SmartDashboard.putNumber("Arm Encoder Ticks", this.arm.returnEncoderTicks());
        SmartDashboard.putNumber("Elevator Encoder Ticks", this.elevator.returnEncoderTicks());
    }

}
