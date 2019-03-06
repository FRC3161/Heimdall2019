package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ManualTurningDriveImpl extends DriveImpl {

    private static final double MANUAL_TURNING_DEADBAND = 0.05;

    public ManualTurningDriveImpl() {
        super();
    }

    @Override
    public void drive(double forwardRate, double strafeRate, double turnRate) {
        double currentAngle = this.angleSensor.pidGet();
        SmartDashboard.putNumber("Gyro:", currentAngle);

        if (this.getCenterWheelsDeployed()) {
            this.tankDrive.arcadeDrive(forwardRate ,turnRate);
        } else {
            final double effectiveTurnRate;
            if (Math.abs(turnRate) < MANUAL_TURNING_DEADBAND) {
                effectiveTurnRate = computedTurnPID;
            } else {
                if (this.turnController.isEnabled()) {
                    this.turnController.disable();
                }
                effectiveTurnRate = turnRate;
            }
            this.holoDrive.driveCartesian(forwardRate, strafeRate, effectiveTurnRate, fieldCentric ? currentAngle : 0);
        }
    }

    @Override
    public void setAngleTarget(double target) {
        super.setAngleTarget(target);
        if (!this.turnController.isEnabled()) {
            this.turnController.enable();
        }
    }
}
