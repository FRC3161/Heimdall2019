package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.SpeedController;

public class ColsonPodImpl implements ColsonPod {

    private final SpeedController controller;

    ColsonPodImpl(int controllerPort) {
        this.controller = new WPI_TalonSRX(controllerPort);
    }

    @Override
    public void setDeployed(boolean deployed) {
        //TODO
    }

    @Override
    public void set(double speed) {
        //TODO
    }

    @Override
    public double get() {
        return 0.0d;
    }

    @Override
    public void disable() {
        //TODO
    }

    @Override
    public void stopMotor() {
        //TODO
    }

    @Override
    public void setInverted(boolean isInverted) {
        //TODO
    }

    @Override
    public boolean getInverted() {
        return true;
    }

    @Override
    public void pidWrite(double output) {
        //TODO
    }
}