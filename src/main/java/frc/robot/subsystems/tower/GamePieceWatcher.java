package frc.robot.subsystems.tower;

import java.util.concurrent.TimeUnit;

import ca.team3161.lib.robot.LifecycleEvent;
import ca.team3161.lib.robot.LifecycleListener;
import ca.team3161.lib.robot.sensors.RightSight;
import ca.team3161.lib.robot.subsystem.RepeatingPooledSubsystem;
import ca.team3161.lib.utils.Utils;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

public class GamePieceWatcher extends RepeatingPooledSubsystem implements LifecycleListener {

    private final RightSight objectSensor;
    private final Relay relay;

    public GamePieceWatcher() {
        super(50, TimeUnit.MILLISECONDS);
        this.relay = Utils.safeInit("Watcher relay", () -> new Relay(RobotMap.LED_SPIKE, Relay.Direction.kForward));
        this.objectSensor = Utils.safeInit("Watcher RightSight", () -> new RightSight(RobotMap.OBJECT_SENSOR));
        this.objectSensor.setInverted(true);
    }

    @Override
    public void defineResources() { }

    @Override
    public void task() {
        boolean detected = this.objectSensor.get();
        SmartDashboard.putBoolean("GamePieceWatcher", detected);
        if (detected) {
            this.relay.set(Relay.Value.kOn);
        } else {
            this.relay.set(Relay.Value.kOff);
        }
    }

    public boolean getObjectState() {
        if (this.objectSensor.get()) {
            return true;
        }
        else{
            return false;
        }
    }

    @Override
    public void lifecycleStatusChanged(LifecycleEvent previous, LifecycleEvent current) {
        switch (current) {
            case ON_TEST:
            case ON_AUTO:
            case ON_TELEOP:
                this.start();
                break;
            default:
                this.cancel();
        }
    }
}