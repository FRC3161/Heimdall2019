package frc.robot.subsystems.tower;

import java.util.concurrent.TimeUnit;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.apache.commons.collections4.BidiMap;
import org.apache.commons.collections4.bidimap.DualHashBidiMap;

import ca.team3161.lib.robot.LifecycleEvent;
import ca.team3161.lib.robot.subsystem.RepeatingPooledSubsystem;
import ca.team3161.lib.utils.Utils;
import ca.team3161.lib.utils.WPISmartPIDTuner;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.TalonPIDSource;
import frc.robot.subsystems.tower.Tower.Position;

class WristImpl extends RepeatingPooledSubsystem implements Wrist, PIDOutput {

    private static final int WRIST_LOWERED_POSITION = -200;

    private final SpeedController controller;
    private final PIDController pid;
    private final TalonPIDSource source;
    private final WPISmartPIDTuner pidTuner;
    private final Supplier<Boolean> armAtTarget;
    private volatile double pidSpeed;
    private volatile boolean manual = true;
    private Position targetPosition = Position.STARTING_CONFIG;
    private volatile double maxOutputUp;
    private volatile double maxOutputDown;

    WristImpl(int victorPort, WPI_TalonSRX sharedTalon, Supplier<Boolean> armAtTarget) {
        super(50, TimeUnit.MILLISECONDS);
        this.controller = Utils.safeInit("wrist", () -> new  VictorSP(victorPort));
        this.source = new TalonPIDSource(sharedTalon);
        this.armAtTarget = armAtTarget;

        final double kP = 0.0058;
        final double kI = 0.0;
        final double kD = 0.005;
        final double kF = 0.0;
        final double ktolerance = 2;
        maxOutputUp = 0.55;
        maxOutputDown = -0.4;
        this.pid = new WPIPIDulum(kP, kI, kD, kF, source, this) {
            @Override
            public double getAngle() {
                // TODO determine actual physical offset from 0 in initialization position
                return super.getAngle();
            }
        };
        this.pid.setAbsoluteTolerance(ktolerance);
        this.pid.setOutputRange(maxOutputDown, maxOutputUp);
        this.pid.setName("Wrist pid");
        this.pidTuner = new WPISmartPIDTuner.Builder()
            .kP(kP)
            .kI(kI)
            .kD(kD)
            .kF(kF)
            .absoluteTolerance(ktolerance)
            .outputRange(maxOutputDown, maxOutputUp)
            .build(pid);

        setPosition(Position.STARTING_CONFIG);
    }

    @Override
    public void setPosition(Position position) {
        this.manual = false;
        this.targetPosition = position;
        int encoderTicks;
        if (position.equals(Position.GROUND)) {
            encoderTicks = WRIST_LOWERED_POSITION;
        } else if (position.equals(Position.LEVEL_1)){
            encoderTicks = 25;
        }
        else {
            encoderTicks = 0;
        }
        SmartDashboard.putNumber("Wrist encoder tick target", encoderTicks);
        this.pid.setSetpoint(encoderTicks);
        this.pid.setEnabled(true);
        SmartDashboard.putString("Wrist Position", position.toString());
    }

    @Override
    public Position getPosition() {
        return this.targetPosition;
    }

    @Override
    public void setSpeed(double speed) {
        this.manual = true;
        this.pid.setEnabled(false);
        if (this.source.pidGet() <= -320) {
            if (speed > 0) {
                speed = 0;
            }
        }
        if (Math.abs(speed) < 0.1) {
            speed = 0;
        }
        this.pidSpeed = speed;
    }

    @Override
    public void reset() {
        this.pid.reset();
        this.source.reset();
        setPosition(Position.STARTING_CONFIG);
    }

    @Override
    public void defineResources() {
        require(pid);
    }

    @Override
    public void lifecycleStatusChanged(LifecycleEvent previous, LifecycleEvent current) {
        this.pidTuner.lifecycleStatusChanged(previous, current);
        if (current.equals(LifecycleEvent.ON_INIT)) {
            start();
        }

        if (previous.equals(LifecycleEvent.ON_AUTO) && current.equals(LifecycleEvent.ON_TELEOP)) {
            return;
        }
        reset();
    }

    @Override
    public void task() {
        SmartDashboard.putNumber("Wrist encoder ticks", this.source.pidGet());
        SmartDashboard.putNumber("Wrist speed", pidSpeed);
        SmartDashboard.putBoolean("Arm atTarget", this.armAtTarget.get());
        if (!manual && targetPosition.equals(Position.BAY) && armAtTarget.get()) {
            int newTickTarget = -210;
            SmartDashboard.putNumber("Wrist encoder tick target", newTickTarget);
            this.pid.setSetpoint(newTickTarget);
        }
        this.controller.set(pidSpeed);
    }

    @Override
    public double returnEncoderTicks() {
        return source.pidGet();
    }

    @Override
    public void pidWrite(double pid) {
        this.pidSpeed = -pid;
    }
}