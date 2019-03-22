package frc.robot.subsystems.tower;

import java.util.concurrent.TimeUnit;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.apache.commons.collections4.BidiMap;
import org.apache.commons.collections4.bidimap.DualHashBidiMap;
import org.apache.commons.collections4.bidimap.UnmodifiableBidiMap;

import ca.team3161.lib.robot.LifecycleEvent;
import ca.team3161.lib.robot.subsystem.RepeatingPooledSubsystem;
import ca.team3161.lib.utils.Utils;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.InvertiblePIDSource;
import frc.robot.subsystems.TalonPIDSource;
import frc.robot.subsystems.tower.Tower.Position;

class WpiPidArmImpl extends RepeatingPooledSubsystem implements Arm, PIDOutput {

    private static final BidiMap<Position, Integer> POSITION_TICKS;
    static {
        final BidiMap<Position, Integer> positionTicks = new DualHashBidiMap<>();
        // TODO placeholder encoder tick values
        positionTicks.put(Position.STARTING_CONFIG, 0);
        positionTicks.put(Position.GROUND, -1);
        positionTicks.put(Position.LEVEL_1, -35 );
        positionTicks.put(Position.LEVEL_2, -153);
        positionTicks.put(Position.LEVEL_3, 6);
        POSITION_TICKS = UnmodifiableBidiMap.unmodifiableBidiMap(positionTicks);
    }

    private final SpeedController controller;
    private final PIDController pid;
    private final PIDSource source;
    private volatile double pidSpeed;
    private Position targetPosition = Position.STARTING_CONFIG;

    WpiPidArmImpl(int talonPort) {
        super(50, TimeUnit.MILLISECONDS);
        WPI_TalonSRX talon = Utils.safeInit("arm controller", () -> new WPI_TalonSRX(talonPort));
        this.controller = talon;
        this.source = new InvertiblePIDSource<PIDSource>(new TalonPIDSource(talon), PIDSource::pidGet);

        final double kP = 0.05;
        final double kI = 0;
        final double kD = 0;
        this.pid = new PIDController(kP, kI, kD, source, this);
    }

    @Override
    public void setPosition(Position position) {
        this.targetPosition = position;
        int encoderTicks;
        if (!POSITION_TICKS.containsKey(this.targetPosition)) {
            encoderTicks = POSITION_TICKS.getOrDefault(Position.STARTING_CONFIG, 0);
        } else {
            encoderTicks = POSITION_TICKS.get(this.targetPosition);
        }
        SmartDashboard.putNumber("encoder tick target arm", encoderTicks);
        this.pid.setSetpoint(encoderTicks);
        SmartDashboard.putString("arm Position", position.toString());
    }

    @Override
    public Position getPosition() {
        return this.targetPosition;
    }

    @Override
    public void setSpeed(double speed) {
        this.controller.set(speed);
    }

    @Override
    public void reset() {
        this.pid.reset();
    }

    @Override
    public void defineResources() { }

    @Override
    public void lifecycleStatusChanged(LifecycleEvent previous, LifecycleEvent current) {
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
        this.controller.set(pidSpeed);
        SmartDashboard.putNumber("arm encoder ticks", this.source.pidGet());
        SmartDashboard.putNumber("arm speed", this.controller.get());
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
