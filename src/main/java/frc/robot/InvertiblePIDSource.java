package frc.robot;

import java.util.function.Function;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class InvertiblePIDSource<T extends PIDSource> implements PIDSource {

    private final T t;
    private final Function<T, Double> accessor;
    private boolean inverted = false;

    public InvertiblePIDSource(T t, Function<T, Double> accessor) {
        this.t = t;
        this.accessor = accessor;
    }

    public void setInverted(boolean inverted) {
        this.inverted = inverted;
    }

    public boolean isInverted() {
        return this.inverted;
    }

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
        t.setPIDSourceType(pidSource);
    }

    @Override
    public double pidGet() {
        return (this.inverted ? -1 : 1) * this.accessor.apply(this.t);
    }

    @Override
    public PIDSourceType getPIDSourceType() {
        return t.getPIDSourceType();
    }
}