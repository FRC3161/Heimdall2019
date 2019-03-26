package frc.robot.subsystems.tower;

import java.util.concurrent.TimeUnit;

import ca.team3161.lib.robot.LifecycleEvent;
import ca.team3161.lib.robot.LifecycleListener;
import ca.team3161.lib.robot.sensors.RightSight;
import ca.team3161.lib.robot.subsystem.RepeatingPooledSubsystem;
import ca.team3161.lib.utils.Utils;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

public class GamePieceWatcher extends RepeatingPooledSubsystem implements LifecycleListener {

    private final RightSight objectSensor;
    private final Relay relay;
    private volatile boolean isTeleop;

    public GamePieceWatcher(Relay relay) {
        super(50, TimeUnit.MILLISECONDS);
        this.relay = relay;
        this.objectSensor = Utils.safeInit("Watcher RightSight", () -> new RightSight(RobotMap.OBJECT_SENSOR));
        this.objectSensor.setInverted(true);
    }

    @Override
    public void defineResources() {
        require(relay);
    }

    @Override
    public void task() {
        if (isTeleop && Timer.getMatchTime() < GameTimerWatcher.ACTIVE_REMAINING_TIME) {
            cancel();
            return;
        }
        boolean detected = this.objectSensor.get();
        SmartDashboard.putBoolean("GamePieceWatcher", detected);
        if (detected) {
            this.relay.set(Relay.Value.kOn);
        } else {
            this.relay.set(Relay.Value.kOff);
        }
    }

    @Override
    public void lifecycleStatusChanged(LifecycleEvent previous, LifecycleEvent current) {
        switch (current) {
            case ON_TEST:
            case ON_AUTO:
                this.isTeleop = false;
                this.start();
                break;
            case ON_TELEOP:
                this.isTeleop = true;
                this.start();
                break;
            default:
                this.cancel();
        }
    }
}