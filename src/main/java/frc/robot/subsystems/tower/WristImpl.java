package frc.robot.subsystems.tower;

import java.util.concurrent.TimeUnit;

import org.apache.commons.collections4.BidiMap;
import org.apache.commons.collections4.bidimap.DualHashBidiMap;

import ca.team3161.lib.robot.LifecycleEvent;
import ca.team3161.lib.robot.subsystem.RepeatingPooledSubsystem;
import ca.team3161.lib.utils.SmartDashboardTuner;
import ca.team3161.lib.utils.Utils;
import ca.team3161.lib.utils.WPISmartPIDTuner;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.tower.Tower.Position;

class WristImpl extends RepeatingPooledSubsystem implements Wrist, PIDOutput {

    private final BidiMap<Position, Integer> positionTicks;

    private final SpeedController controller;
    private final PIDController pid;
    private final Encoder source;
    private final WPISmartPIDTuner pidTuner;
    private final SmartDashboardTuner levelOneTuner;
    private final SmartDashboardTuner levelTwoTuner;
    private final SmartDashboardTuner bayTuner;
    private volatile double pidSpeed;
    private volatile boolean manual = true;
    private Position targetPosition = Position.STARTING_CONFIG;
    private volatile double maxOutputUp;
    private volatile double maxOutputDown;

    WristImpl(int victorPort, int encoderChannelA, int encoderChannelB) {
        super(50, TimeUnit.MILLISECONDS);
        this.controller = Utils.safeInit("wrist", () -> new  VictorSP(victorPort));
        this.controller.setInverted(true);
        this.source = new Encoder(encoderChannelA, encoderChannelB);

        final int levelOneTicks = 50;
        final int levelTwoTicks = 153;
        final int bayTicks = 120;
        positionTicks = new DualHashBidiMap<>();
        positionTicks.put(Position.STARTING_CONFIG, 0);
        positionTicks.put(Position.GROUND, 1);
        positionTicks.put(Position.LEVEL_1, levelOneTicks);
        positionTicks.put(Position.BAY, bayTicks);
        positionTicks.put(Position.LEVEL_2, levelTwoTicks);
        positionTicks.put(Position.LEVEL_3, 6);

        final double kP = 0.0225;
        final double kI = 0.0001;
        final double kD = 0.0285;
        final double ktolerance = 2;
        maxOutputUp = 0.65;
        maxOutputDown = -0.275;
        this.pid = new PIDController(kP, kI, kD, source, this);
        this.pid.setAbsoluteTolerance(ktolerance);
        this.pid.setOutputRange(maxOutputDown, maxOutputUp);
        this.pid.setName("Wrist pid");
        this.pidTuner = new WPISmartPIDTuner.Builder()
            .kP(kP)
            .kI(kI)
            .kD(kD)
            .absoluteTolerance(ktolerance)
            .outputRange(maxOutputDown, maxOutputUp)
            .build(pid);

        this.levelOneTuner = new SmartDashboardTuner("Level One Ticks", levelOneTicks, d -> positionTicks.put(Position.LEVEL_1, d.intValue()));
        this.levelTwoTuner = new SmartDashboardTuner("Level Two Ticks", levelTwoTicks, d -> positionTicks.put(Position.LEVEL_2, d.intValue()));
        this.bayTuner = new SmartDashboardTuner("bay Ticks", bayTicks, d -> positionTicks.put(Position.BAY, d.intValue()));
    }

    @Override
    public void setPosition(Position position) {
        this.manual = false;
        this.targetPosition = position;
        int encoderTicks;
        if (!positionTicks.containsKey(this.targetPosition)) {
            encoderTicks = positionTicks.getOrDefault(Position.STARTING_CONFIG, 0);
        } else {
            encoderTicks = positionTicks.get(this.targetPosition);
        }
        SmartDashboard.putNumber("encoder tick target Wrist", encoderTicks);
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
        if (speed < maxOutputDown) {
            speed = maxOutputDown;
        } else if (speed > maxOutputUp) {
            speed = maxOutputUp;
        }
        this.controller.set(speed);
    }

    @Override
    public void reset() {
        this.pid.reset();
        this.source.reset();
    }

    @Override
    public void defineResources() {
        require(pid);
    }

    @Override
    public void lifecycleStatusChanged(LifecycleEvent previous, LifecycleEvent current) {
        this.pidTuner.lifecycleStatusChanged(previous, current);
        this.levelOneTuner.lifecycleStatusChanged(previous, current);
        this.levelTwoTuner.lifecycleStatusChanged(previous, current);
        this.bayTuner.lifecycleStatusChanged(previous, current);
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
        SmartDashboard.putNumber("Wrist speed", this.controller.get());
        if (this.manual) {
            if (Math.abs(controller.get()) < 0.1) {
                // this.pid.setSetpoint(this.returnEncoderTicks());
                // this.manual = false;
                this.controller.set(0);
                return;
            }
        }
        this.controller.set(pidSpeed);
    }

    @Override
    public double returnEncoderTicks() {
        return source.pidGet();
    }

    @Override
    public void pidWrite(double pid) {
        this.pidSpeed = pid;
    }
}