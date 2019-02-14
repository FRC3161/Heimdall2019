package frc.robot.subsystems.tower;

import org.apache.commons.collections4.BidiMap;
import org.apache.commons.collections4.bidimap.DualHashBidiMap;
import org.apache.commons.collections4.bidimap.UnmodifiableBidiMap;

import ca.team3161.lib.robot.LifecycleEvent;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.tower.Tower.Position;
import frc.robot.subsystems.Gains;

class ElevatorImpl implements Elevator {

    private static final BidiMap<Position, Integer> POSITION_TICKS;
    static {
        final BidiMap<Position, Integer> positionTicks = new DualHashBidiMap<>();
        // TODO placeholder encoder tick values
        positionTicks.put(Position.STARTING_CONFIG, 0);
        positionTicks.put(Position.LOW, -10000);
        positionTicks.put(Position.CARGO_2, -15000);
        positionTicks.put(Position.CARGO_3, -25000);
        positionTicks.put(Position.HATCH_2, -30000);
        positionTicks.put(Position.HATCH_3, -40000);
        POSITION_TICKS = UnmodifiableBidiMap.unmodifiableBidiMap(positionTicks);
    }

    private final WPI_TalonSRX controllerMaster;
    private final WPI_TalonSRX controllerSlave;
    private final int kPIDLoopIdx = 0;
    private final DigitalInput limitSwitchTop;
    private final DigitalInput limitSwitchBottom;

    private Position targetPosition = Position.STARTING_CONFIG;

    ElevatorImpl(int masterPort, int slavePort, int topSwitchPort , int bottomSwitchPort) {
        this.controllerMaster = new WPI_TalonSRX(masterPort);
        this.controllerSlave = new WPI_TalonSRX(slavePort);
        this.controllerSlave.follow(controllerMaster);
        this.limitSwitchTop = new DigitalInput(topSwitchPort);
        this.limitSwitchBottom = new DigitalInput(bottomSwitchPort);

        //Arm PID
        final Gains kGains = new Gains(0.75, 0.08, 0.25, 0.0, 0, 0.75); //TODO Placeholder values
        final int kTimeoutMs = 30;
        // final boolean kSensorPhase = true;
        // final boolean kMotorInvert = false;
        // int absolutePosition = controllerMaster.getSensorCollection().getPulseWidthPosition();

        //Set PID values on Talon
        controllerMaster.config_kF(kPIDLoopIdx, kGains.kF);
        controllerMaster.config_kP(kPIDLoopIdx, kGains.kP);
        controllerMaster.config_kI(kPIDLoopIdx, kGains.kI);
        controllerMaster.config_kD(kPIDLoopIdx, kGains.kD);
        controllerMaster.setInverted(false);
        controllerMaster.setSensorPhase(false);
        controllerMaster.configAllowableClosedloopError(kPIDLoopIdx, 150);

        // absolutePosition &= 0xFFF;
        // if (kSensorPhase) {
        //     absolutePosition *= -1;
        // }
        // if (kMotorInvert) {
        //     absolutePosition *= -1;
        // }

        controllerMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, kPIDLoopIdx, kTimeoutMs);
        controllerMaster.setSelectedSensorPosition(0, 0, 0);

        //Speed Limiting
        controllerMaster.configPeakOutputForward(kGains.kPeakOutput);
        controllerMaster.configPeakOutputReverse(-kGains.kPeakOutput);
    }

    @Override
    public void setPosition(Position position) {
        this.targetPosition = position;
        int encoderTicks;
        if (!POSITION_TICKS.containsKey(position)) {
            encoderTicks = POSITION_TICKS.get(Position.STARTING_CONFIG);
        } else {
            encoderTicks = POSITION_TICKS.get(position);
        }

        this.controllerMaster.set(ControlMode.Position, encoderTicks);
    }

    @Override
    public Position getPosition() {
        return this.targetPosition;
    }

    @Override
    public void setSpeed(double speed) {
        // if (limitSwitchTop.get()) {
        //     if (speed > 0){
        //         speed = 0;
        //     }
        // } else if (limitSwitchBottom.get()) {
        //     if (speed < 0){
        //         speed = 0;
        //     }
        // }
        controllerMaster.set(ControlMode.PercentOutput, speed);
        SmartDashboard.putBoolean("bottom elevator limit", limitSwitchBottom.get());
        SmartDashboard.putNumber("elevator speed", controllerMaster.get());
        SmartDashboard.putNumber("elevator encoder ticks", controllerMaster.getSelectedSensorPosition());
    }

    @Override
    public void reset() {
        controllerMaster.setSelectedSensorPosition(0);
    }

    @Override
    public void lifecycleStatusChanged(LifecycleEvent previous, LifecycleEvent current) {
        if (previous.equals(LifecycleEvent.ON_AUTO) && current.equals(LifecycleEvent.ON_TELEOP)){
            return;
        }
        reset();
    }

}