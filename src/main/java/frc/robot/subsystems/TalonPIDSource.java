package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import java.util.function.Function;

public class TalonPIDSource implements PIDSource {
    private final WPI_TalonSRX talon;
	    private final int sensorIndex;
	    private PIDSourceType type = PIDSourceType.kDisplacement;
	
	    public TalonPIDSource(final WPI_TalonSRX talon, final int sensorIndex) {
	        this.talon = talon;
	        this.sensorIndex = sensorIndex;
	    }
	
	    public TalonPIDSource(final WPI_TalonSRX talon) {
	        this(talon, 0);
	    }
	
	    @Override
	    public void setPIDSourceType(PIDSourceType pidSourceType) {
	        this.type = pidSourceType;
	    }
	
	    @Override
	    public PIDSourceType getPIDSourceType() {
	        return this.type;
	    }
	
	    @Override
	    public double pidGet() {
	        final Function<Integer, Integer> producer;
	        switch (this.type) {
	            case kRate:
	                producer = this.talon::getSelectedSensorVelocity;
	                break;
	            case kDisplacement:
	                producer = this.talon::getSelectedSensorPosition;
	                break;
	            default:
	                producer = (idx) -> 0;
	                break;
	        }
	        return producer.apply(this.sensorIndex);
}
}