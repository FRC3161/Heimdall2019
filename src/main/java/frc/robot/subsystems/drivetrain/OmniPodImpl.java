package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import frc.robot.subsystems.TalonPIDSource;
import frc.robot.subsystems.TalonRatePIDSource;

// TODO implement PID control over the contained Talon with its attached encoder
public class OmniPodImpl implements OmniPod {

    private final WPI_TalonSRX talon;

    private final TalonPIDSource talonPIDSource;
    private final PIDController talonPIDController;

    public OmniPodImpl(int talonCANPort) {
        this(talonCANPort, 0.001, 0.001, 0.001, 100);
    }

    public OmniPodImpl(int talonCANPort, double Kp, double Ki, double Kd, double maxRotationalRate) {
        this.talon = new WPI_TalonSRX(talonCANPort);
        this.talonPIDSource = new TalonRatePIDSource(talon, maxRotationalRate);
        this.talonPIDSource.setPIDSourceType(PIDSourceType.kRate);
        this.talonPIDController = new PIDController(Kp, Ki, Kd, talonPIDSource, this);
    }

    @Override
    public void set(double speed) {
        if (!this.talonPIDController.isEnabled()) {
            this.talonPIDController.enable();
        };
        this.talonPIDController.setSetpoint(speed);
    }

    @Override
    public double get() {
        return this.talon.get();
    }

    @Override
    public void disable() {
        this.talon.disable();
        this.talonPIDController.reset();
    }

    @Override
    public void stopMotor() {
        this.talon.stopMotor();
        this.talonPIDController.reset();
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

}