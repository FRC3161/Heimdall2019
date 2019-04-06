package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.MathUtils;

public class ManualTurningDriveImpl extends DriveImpl {

    private static final double MANUAL_TURNING_DEADBAND = 0.05;
    private static final long UPDATE_WINDOW = 100;
    private long lastUpdate = -1;

    public ManualTurningDriveImpl() {
        super();
    }

    @Override
    public void drive(double forwardRate, double strafeRate, double turnRate) {
        double currentAngle = this.angleSensor.pidGet();
        SmartDashboard.putNumber("Gyro:", currentAngle);

        if (this.speedLimited) {
            final double limit = 0.45;
            forwardRate = MathUtils.absClamp(forwardRate, limit);
            strafeRate = MathUtils.absClamp(strafeRate, limit);
            turnRate = MathUtils.absClamp(turnRate, limit);
        }

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
        long now = System.currentTimeMillis();
        if (lastUpdate + UPDATE_WINDOW > now) {
            return;
        }
        lastUpdate = now;
        super.setAngleTarget(target);
        if (!this.turnController.isEnabled()) {
            this.turnController.enable();
        }
    }
}
