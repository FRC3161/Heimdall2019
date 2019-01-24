package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.PIDSourceType;

public class TalonRatePIDSource extends TalonPIDSource {

    private final double maxRotationalRate;

    public TalonRatePIDSource(final WPI_TalonSRX talon, final int sensorIndex, final double maxRotationalRate) {
        super(talon, sensorIndex);
        this.setPIDSourceType(PIDSourceType.kRate);
        this.maxRotationalRate = maxRotationalRate;
    }

    public TalonRatePIDSource(final WPI_TalonSRX talon, final double maxRotationalRate) {
        this(talon, 0, maxRotationalRate);
    }

    @Override
    public double pidGet() {
        return super.pidGet() / this.maxRotationalRate;
    }
}