package frc.robot.subsystems.tower;

import java.util.concurrent.TimeUnit;

import ca.team3161.lib.robot.LifecycleEvent;
import ca.team3161.lib.robot.LifecycleListener;
import ca.team3161.lib.robot.subsystem.RepeatingPooledSubsystem;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Timer;

public class GameTimerWatcher extends RepeatingPooledSubsystem implements LifecycleListener {

    static final double ACTIVE_REMAINING_TIME = 30;
    private static final double SOLID_REMAINING_TIME = 20;
    private final Relay relay;
    private boolean lit = false;

    public GameTimerWatcher(Relay relay) {
        super(500, TimeUnit.MILLISECONDS);
        this.relay = relay;
    }

    @Override
    public void defineResources() {
        require(relay);
    }

    @Override
    public void task() {
        if (Timer.getMatchTime() < ACTIVE_REMAINING_TIME && Timer.getMatchTime() > SOLID_REMAINING_TIME) {
            lit = !lit;
            this.relay.set(lit ? Relay.Value.kOn : Relay.Value.kOff);
        } else if (Timer.getMatchTime() <= SOLID_REMAINING_TIME) {
            this.relay.set(Relay.Value.kOn);
            cancel();
        }
    }

    @Override
    public void lifecycleStatusChanged(LifecycleEvent previous, LifecycleEvent current) {
        switch (current) {
            case ON_TEST:
            case ON_TELEOP:
                this.start();
                break;
            default:
                this.cancel();
        }
    }
}
