package frc.robot.subsystems.tower;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import ca.team3161.lib.robot.LifecycleEvent;
import ca.team3161.lib.utils.Utils;
import frc.robot.subsystems.tower.Tower.Position;

public class ManualElevatorImpl implements Elevator {

    private final WPI_TalonSRX controllerMaster;
    private final WPI_TalonSRX controllerSlave;

    // slaveTalon is a hack to allow the Wrist to repurpose one of the elevator Talons'
    // onboard sensor ports for its encoder
    ManualElevatorImpl(int masterPort, WPI_TalonSRX slaveTalon) {
        this.controllerMaster = Utils.safeInit("elevator controllerMaster", () -> new WPI_TalonSRX(masterPort));
        this.controllerSlave = slaveTalon;
        this.controllerSlave.follow(controllerMaster);

        controllerMaster.setInverted(false);
    }

    @Override
    public void setPosition(Position position) { }

    @Override
    public Position getPosition() {
        return Position.STARTING_CONFIG;
    }

    @Override
    public void setSpeed(double speed) {
        controllerMaster.set(ControlMode.PercentOutput, speed);
    }

    @Override
    public void reset() { }

    @Override
    public void lifecycleStatusChanged(LifecycleEvent previous, LifecycleEvent current) { }

    @Override
    public double returnEncoderTicks() {
        return 1.d;
    }

}