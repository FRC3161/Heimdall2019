package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;

public class ColsonPodImpl implements ColsonPod {

    private final SpeedController controller;
    private final Solenoid solenoid;

    ColsonPodImpl(int controllerPort, Solenoid solenoid) {
        this.controller = new WPI_TalonSRX(controllerPort);
        this.solenoid = solenoid;
    }

    @Override
    public void setDeployed(boolean deployed) {
        this.solenoid.set(deployed);
    }

    @Override
    public boolean isDeployed() {
        return this.solenoid.get();
    }

    @Override
    public void set(double speed) {
        this.controller.set(speed);
    }

    @Override
    public double get() {
        return this.controller.get();
    }

    @Override
    public void disable() {
        this.controller.disable();
        this.setDeployed(false);
    }

    @Override
    public void stopMotor() {
        this.controller.stopMotor();
        this.setDeployed(false);
    }

    @Override
    public void setInverted(boolean isInverted) {
        this.controller.setInverted(isInverted);
    }

    @Override
    public boolean getInverted() {
        return this.controller.getInverted();
    }

    @Override
    public void pidWrite(double output) {
        this.controller.pidWrite(output);
    }
}