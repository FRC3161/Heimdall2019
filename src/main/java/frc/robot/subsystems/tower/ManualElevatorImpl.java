package frc.robot.subsystems.tower;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import ca.team3161.lib.robot.LifecycleEvent;
import ca.team3161.lib.utils.Utils;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.tower.Tower.Position;

public class ManualElevatorImpl implements Elevator {

    private final WPI_TalonSRX controllerMaster;
    private final WPI_TalonSRX controllerSlave;
    private final DigitalInput limitSwitchTop;
    private final DigitalInput limitSwitchBottom;

    ManualElevatorImpl(int masterPort, int slavePort, int topSwitchPort , int bottomSwitchPort) {
        this.controllerMaster = Utils.safeInit("elevator controllerMaster", () -> new WPI_TalonSRX(masterPort));
        this.controllerSlave = Utils.safeInit("elevator controllerSlave", () -> new WPI_TalonSRX(slavePort));
        this.limitSwitchBottom = Utils.safeInit("elevator limitSwitchBottom", () -> new DigitalInput(bottomSwitchPort));
        this.limitSwitchTop = Utils.safeInit("elevator limitSwitchTop", () -> new DigitalInput(topSwitchPort));
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