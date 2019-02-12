package frc.robot.subsystems.tower;

import static frc.robot.MathUtils.absClamp;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.Encoder;

import org.apache.commons.collections4.BidiMap;
import org.apache.commons.collections4.bidimap.DualHashBidiMap;
import org.apache.commons.collections4.bidimap.UnmodifiableBidiMap;

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

    // TODO set up TalonSRX for Position PID:
    // https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java/PositionClosedLoop/src/main/java/frc/robot/Robot.java
    private final WPI_TalonSRX controller;
    private Position targetPosition = Position.STARTING_CONFIG;

    //Arm PID
    private final int kPIDLoopIdx;
    private final Gains kGains;
    private final int kTimeoutMs;

    ArmImpl(int talonPort) {
        this.controller = new WPI_TalonSRX(talonPort);
        double motoroutput = this.controller.getMotorOutputPercent();
        this.kPIDLoopIdx = 0;
        this.kGains = new Gains(0.15, 0.17, 0.16, 0.0, 0, 1.0); //TODO Placeholder values
        this.kTimeoutMs = 30;
        controller.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, kPIDLoopIdx, kTimeoutMs);
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
        // TODO do something useful with encoder ticks
    }

    @Override
    public Position getPosition() {
        return this.targetPosition;
    }

    @Override
    public void setSpeed(double speed) {
        double maxPower= 0.25;
        this.controller.set(absClamp(speed, maxPower));
    }
}