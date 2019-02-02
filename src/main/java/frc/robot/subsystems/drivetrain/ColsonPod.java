package frc.robot.subsystems.drivetrain;

public interface ColsonPod extends WheelPod {
    void setDeployed(boolean deployed);
    boolean isDeployed();
}