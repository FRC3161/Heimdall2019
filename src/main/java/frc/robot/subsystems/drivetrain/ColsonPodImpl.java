package frc.robot.subsystems.drivetrain;

import ca.team3161.lib.utils.Utils;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.VictorSP;

public class ColsonPodImpl implements ColsonPod {

    private final SpeedController controller;
    private final Solenoid solenoid;

    ColsonPodImpl(int controllerPort, Solenoid solenoid) {
        this.controller = Utils.safeInit("Colson pod port #" + controllerPort, () -> new VictorSP(controllerPort));
        this.solenoid = solenoid;
    }

    @Override
    public void setDeployed(boolean deployed) {
        this.solenoid.set(deployed);
        if (!deployed) {
            this.controller.stopMotor();
        }
    }

    @Override
    public boolean isDeployed() {
        return this.solenoid.get();
    }

    @Override
    public void set(double speed) {
        if (isDeployed()) {
            this.controller.setInverted(true);
            this.controller.set(speed);
        }
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
        if (isDeployed()) {
            this.controller.pidWrite(output);
        }
    }
}