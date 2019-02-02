package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * OmniPod without PID
 */
public class RawOmniPodImpl implements OmniPod {
    
    private static final double SCALE_DOWN_FACTOR = Math.sqrt(2);

    protected final WPI_TalonSRX talon;
    private boolean scaledDown = false;

    public RawOmniPodImpl(int talonCANPort) {
        this.talon = new WPI_TalonSRX(talonCANPort);
    }

    @Override
    public void set(double speed) {
        this.talon.set(speed);
    }

    @Override
    public double get() {
        return this.talon.get();
    }

    @Override
    public void disable() {
        this.talon.disable();
    }

    @Override
    public void stopMotor() {
        this.talon.stopMotor();
    }

    @Override
    public void setInverted(boolean isInverted) {
        this.talon.setInverted(isInverted);
    }

    @Override
    public boolean getInverted() {
        return this.talon.getInverted();
    }

    @Override
    public void pidWrite(double output) {
        this.talon.pidWrite(output);
    }
    
    @Override
    public void setScaledDown(boolean scaledDown) {
        this.scaledDown = scaledDown;
    }

    @Override
    public boolean isScaledDown() {
        return this.scaledDown;
    }

    protected double scale(double speed) {
        return speed / (this.isScaledDown() ? SCALE_DOWN_FACTOR : 1);
    }
}