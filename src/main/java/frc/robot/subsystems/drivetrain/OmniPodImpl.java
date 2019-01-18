package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.PIDController;
import frc.robot.subsystems.TalonPIDSource;

// TODO implement PID control over the contained Talon with its attached encoder
public class OmniPodImpl implements OmniPod {

    private final WPI_TalonSRX talon;
    
    //PID
    private double Kp;
    private double Ki;
    private double Kd;

    TalonPIDSource talonPIDSource;
    PIDController talonPIDController;
    //PIDOutput talonPIDOutput;

    public OmniPodImpl(int talonCANPort) {
        this.Kp = 0.001;
        this.Ki = 0.001;
        this.Kd = 0.001;
        
        this.talon = new WPI_TalonSRX(talonCANPort);

        this.talonPIDSource = new TalonPIDSource(talon);
        this.talonPIDController = new PIDController(Kp, Ki, Kd, talonPIDSource, talon);
    }

    public OmniPodImpl(int talonCANPort, double Kp, double Ki, double Kd){
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        
        this.talon = new WPI_TalonSRX(talonCANPort);

        this.talonPIDSource = new TalonPIDSource(talon);
        this.talonPIDController = new PIDController(Kp, Ki, Kd, talonPIDSource, talon);
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

}