package frc.robot.subsystems.drivetrain;

public interface Drive {
    void drive(double forwardRate, double strafeRate, double turnRate);
    void setCenterWheelsDeployed(boolean deployed);
    boolean getCenterWheelsDeployed();
    void resetGyro();
}