package frc.robot.subsystems.tower;

import org.apache.commons.collections4.BidiMap;
import org.apache.commons.collections4.bidimap.DualHashBidiMap;
import org.apache.commons.collections4.bidimap.UnmodifiableBidiMap;

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
        positionTicks.put(Position.LOW, 1);
        positionTicks.put(Position.CARGO_2, 2);
        positionTicks.put(Position.CARGO_3, 3);
        positionTicks.put(Position.HATCH_2, 4);
        positionTicks.put(Position.HATCH_3, 5);
        POSITION_TICKS = UnmodifiableBidiMap.unmodifiableBidiMap(positionTicks);
    }

    private final WPI_TalonSRX controllerMaster;
    private final WPI_TalonSRX controllerSlave;
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
        final int kPIDLoopIdx = 1;
        final Gains kGains = new Gains(0.15, 0.0, 0.0, 0.0, 0, 0.75); //TODO Placeholder values
        final int kTimeoutMs = 30;
        final boolean kSensorPhase = true;
        final boolean kMotorInvert = false;
        int absolutePosition = controllerMaster.getSensorCollection().getPulseWidthPosition();

        //Set PID values on Talon
        controllerMaster.config_kF(kPIDLoopIdx, kGains.kF);
        controllerMaster.config_kP(kPIDLoopIdx, kGains.kP);
        controllerMaster.config_kI(kPIDLoopIdx, kGains.kI);
        controllerMaster.config_kD(kPIDLoopIdx, kGains.kD);

        absolutePosition &= 0xFFF;
        if (kSensorPhase) {
            absolutePosition *= -1;
        }
        if (kMotorInvert) {
            absolutePosition *= -1;
        }

        controllerMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, kPIDLoopIdx, kTimeoutMs);
        controllerMaster.setSelectedSensorPosition(absolutePosition, kPIDLoopIdx, kTimeoutMs);

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
        if (limitSwitchTop.get()) {
            if (speed > 0){
                speed = 0;
            }
        } else if (limitSwitchBottom.get()) {
            if (speed < 0){
                speed = 0;
            }
        }
        controllerMaster.set(speed);
        SmartDashboard.putBoolean("bottom elevator limit", limitSwitchBottom.get());
        SmartDashboard.putNumber("talon speed master", controllerMaster.get());
        SmartDashboard.putNumber("talon speed slave", controllerSlave.get());
        SmartDashboard.putNumber("elevator encoder ticks", controllerMaster.getSelectedSensorPosition());
    }

}