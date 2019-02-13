package frc.robot.subsystems.tower;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.apache.commons.collections4.BidiMap;
import org.apache.commons.collections4.bidimap.DualHashBidiMap;
import org.apache.commons.collections4.bidimap.UnmodifiableBidiMap;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.subsystems.tower.Tower.Position;
import frc.robot.subsystems.Gains;

class ArmImpl implements Arm {

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

    private final WPI_TalonSRX controller;
    private Position targetPosition = Position.STARTING_CONFIG;

    ArmImpl(int talonPort) {
        this.controller = new WPI_TalonSRX(talonPort);
        
        //Arm PID
        final int kPIDLoopIdx;
        final Gains kGains;
        final int kTimeoutMs;
        boolean kSensorPhase;
        boolean kMotorInvert;
        int absolutePosition;
        
        kPIDLoopIdx = 0;
        kGains = new Gains(0.15, 0.17, 0.16, 0.0, 0, 1.0); //TODO Placeholder values
        kTimeoutMs = 30;
        absolutePosition = controller.getSensorCollection().getPulseWidthPosition();
        kMotorInvert = false;
        kSensorPhase = true;

        //Set PID values on Talon
        controller.config_kF(kPIDLoopIdx, kGains.kF);
        controller.config_kP(kPIDLoopIdx, kGains.kP);
        controller.config_kI(kPIDLoopIdx, kGains.kI);
        controller.config_kD(kPIDLoopIdx, kGains.kD);
        
        absolutePosition &= 0xFFF;
        if (kSensorPhase) {absolutePosition *= -1;}
        if (kMotorInvert) {absolutePosition *= -1;}

        controller.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, kPIDLoopIdx, kTimeoutMs);
        controller.setSelectedSensorPosition(absolutePosition, kPIDLoopIdx, kTimeoutMs);

        //Speed Limiting
        double maxPower = 0.25;
        controller.configPeakOutputForward(maxPower);
        controller.configPeakOutputReverse(maxPower);

        //controller.configAllowableClosedloopError(kPIDLoopIdx, allowableCloseLoopError, kTimeoutMs);
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
        this.controller.set(ControlMode.Position, encoderTicks);
    }

    @Override
    public Position getPosition() {
        return this.targetPosition;
    }

    @Override
    public void setSpeed(double speed) {
        this.controller.set(speed);
    }
}