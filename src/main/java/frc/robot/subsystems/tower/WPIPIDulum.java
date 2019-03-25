package frc.robot.subsystems.tower;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;

class WPIPIDulum extends PIDController {

    WPIPIDulum(double kP, double kI, double kD, double kF, PIDSource source,
            PIDOutput output) {
        super(kP, kI, kD, kF, source, output);
    }

    @Override
    protected double calculateFeedForward() {
        return getF() * Math.sin(Math.toRadians(getAngle()));
    }

    /**
     * Converts the source reading to a physical angle. This default implementation
     * simply assumes that the source sensor already reports the physical angle, where
     * angle 0 corresponds to the direction of the gravity vector as well as the mechanism
     * pointing straight downward. Angle 180 would correspond to the direction straight upward,
     * and angle 90 corresponds to the mechanism sitting parallel to the ground (ie maximum
     * gravitational torque).
     * Subclasses should override this method to scale and offset the sensor reading if necessary.
     */
    public double getAngle() {
        return m_pidInput.pidGet();
    }

}