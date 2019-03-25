package frc.robot.subsystems.drivetrain;

public interface StarDrive {
    void drive(double forwardRate, double strafeRate, double turnRate);

    void setAngleTarget(double angleTarget);

    void setCenterWheelsDeployed(boolean deployed);
    boolean getCenterWheelsDeployed();
    default void toggleCenterWheelsDeployed() {
        setCenterWheelsDeployed(!getCenterWheelsDeployed());
    }

    void resetGyro();

    void setFieldCentric(boolean enabled);
    boolean isFieldCentric();

    void resetTurnController();
}
