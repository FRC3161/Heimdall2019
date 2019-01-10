package frc.robot.subsystems;

public interface Drive {
    void drive(double forwardRate, double strafeRate, double turnRate);
    void setCenterWheelsDeployed(boolean deployed);
    boolean getCenterWheelsDeployed();
}