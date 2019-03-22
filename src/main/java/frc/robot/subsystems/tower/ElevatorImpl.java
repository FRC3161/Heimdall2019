package frc.robot.subsystems.tower;

import org.apache.commons.collections4.BidiMap;
import org.apache.commons.collections4.bidimap.DualHashBidiMap;
import org.apache.commons.collections4.bidimap.UnmodifiableBidiMap;

import ca.team3161.lib.robot.LifecycleEvent;
import ca.team3161.lib.robot.subsystem.RepeatingPooledSubsystem;
import ca.team3161.lib.utils.Utils;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import java.util.Objects;
import java.util.concurrent.TimeUnit;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.tower.Tower.Position;
import frc.robot.subsystems.Gains;

class ElevatorImpl extends RepeatingPooledSubsystem implements Elevator {
    private final WPI_TalonSRX controllerMaster;
    private final WPI_TalonSRX controllerSlave;
    private final DigitalInput limitSwitchTop;
    private final DigitalInput limitSwitchBottom;
    private final int kPIDLoopIdx = 0;

    private Position targetPosition = Position.STARTING_CONFIG;

    ElevatorImpl(int masterPort, int slavePort, int topSwitchPort,int bottomSwitchPort) {
        super(50, TimeUnit.MILLISECONDS);
        this.controllerMaster = Utils.safeInit("elevator controllerMaster", () -> new WPI_TalonSRX(masterPort));
        this.controllerSlave = Utils.safeInit("elevator controllerSlave", () -> new WPI_TalonSRX(slavePort));
        this.limitSwitchBottom = Utils.safeInit("elevator limitSwitchBottom", () -> new DigitalInput(bottomSwitchPort));
        this.limitSwitchTop = Utils.safeInit("elevator limitSwitchTop", () -> new DigitalInput(topSwitchPort));
        this.controllerSlave.follow(controllerMaster);
        //Arm PID
        final Gains kGains = new Gains(0.001, 0.001, 0.001, 0.0, 0, 1); //TODO Placeholder values
        final int kTimeoutMs = 30;
        // final boolean kSensorPhase = true;
        // final boolean kMotorInvert = false;
        // int absolutePosition = controllerMaster.getSensorCollection().getPulseWidthPosition();

        //Set PID values on Talon
        // controllerMaster.config_kF(kPIDLoopIdx, kGains.kF);
        // controllerMaster.config_kP(kPIDLoopIdx, kGains.kP);
        // controllerMaster.config_kI(kPIDLoopIdx, kGains.kI);
        // controllerMaster.config_kD(kPIDLoopIdx, kGains.kD);
        // controllerMaster.setInverted(false);
        // controllerMaster.setSensorPhase(false);
        // controllerMaster.configAllowableClosedloopError(kPIDLoopIdx, 150);

        // // absolutePosition &= 0xFFF;
        // // if (kSensorPhase) {
        // //     absolutePosition *= -1;
        // // }
        // // if (kMotorInvert) {
        // //     absolutePosition *= -1;
        // // }

        // controllerMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, kPIDLoopIdx, kTimeoutMs);
        // controllerMaster.setSelectedSensorPosition(0, 0, 0);

        // //Speed Limiting
        // controllerMaster.configPeakOutputForward(kGains.kPeakOutput);
        // controllerMaster.configPeakOutputReverse(-kGains.kPeakOutput);
        
    }

    @Override
    public void setPosition(Position position) {
        if (Objects.equals(this.targetPosition, position)) {
            // ignore repeat position requests
            return;
        }
        this.targetPosition = position;
        int encoderTicks;
        if (position.equals(Position.LEVEL_3)) {
            encoderTicks = -40000;
        } else {
            encoderTicks = 0;
        }

        // this.controllerMaster.setIntegralAccumulator(0);
        // this.controllerMaster.set(ControlMode.Position, encoderTicks);
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
        }else if (limitSwitchBottom.get()) {
            if (speed < 0){
                 speed = 0;
            }
        }
        // controllerMaster.set(ControlMode.PercentOutput, speed);
    }

    @Override
    public void reset() {
        // controllerMaster.setSelectedSensorPosition(0);
        // controllerMaster.setIntegralAccumulator(0);
    }

    @Override
    public void lifecycleStatusChanged(LifecycleEvent previous, LifecycleEvent current) {
        if (previous.equals(LifecycleEvent.ON_AUTO) && current.equals(LifecycleEvent.ON_TELEOP)){
            return;
        }
        reset();
    }

    @Override
    public void task() throws Exception {
        SmartDashboard.putNumber("elevator motor output", controllerMaster.getMotorOutputPercent());
        SmartDashboard.putNumber("elevator speed", controllerMaster.getSelectedSensorVelocity());
        SmartDashboard.putNumber("elevator encoder ticks", controllerMaster.getSelectedSensorPosition());
        SmartDashboard.putNumber("elevator integral windup", controllerMaster.getIntegralAccumulator());
    }

    @Override
    public void defineResources() { }

    @Override
    public double returnEncoderTicks() {
        return controllerMaster.getSelectedSensorPosition();
    }

}
