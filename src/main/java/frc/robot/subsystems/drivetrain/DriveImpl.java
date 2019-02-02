package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;

import com.kauailabs.navx.frc.AHRS;

import ca.team3161.lib.robot.motion.drivetrains.SpeedControllerGroup;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveImpl implements Drive {
    private final MecanumDrive holoDrive;
    private final DifferentialDrive tankDrive;
    private final OmniPod frontLeftDrive;
    private final OmniPod frontRightDrive;
    private final OmniPod backLeftDrive;
    private final OmniPod backRightDrive;
    private final ColsonPod leftColson;
    private final ColsonPod rightColson;

    private AHRS ahrs;

    public DriveImpl() {
        this.frontLeftDrive = new RawOmniPodImpl(RobotMap.DRIVETRAIN_LEFT_FRONT_TALON);
        frontLeftDrive.setInverted(true);
        this.frontRightDrive = new RawOmniPodImpl(RobotMap.DRIVETRAIN_RIGHT_FRONT_TALON);
        frontRightDrive.setInverted(false);
        this.backLeftDrive = new RawOmniPodImpl(RobotMap.DRIVETRAIN_LEFT_BACK_TALON);
        backLeftDrive.setInverted(false);
        this.backRightDrive = new RawOmniPodImpl(RobotMap.DRIVETRAIN_RIGHT_BACK_TALON);
        backRightDrive.setInverted(true);

        Solenoid colsonValve = new Solenoid(0);
        this.leftColson = new ColsonPodImpl(RobotMap.DRIVETRAIN_LEFT_COLSON, colsonValve);
        this.rightColson = new ColsonPodImpl(RobotMap.DRIVETRAIN_RIGHT_COLSON, colsonValve);

        this.holoDrive = new MecanumDrive(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);
        this.tankDrive = new DifferentialDrive(
            new SpeedControllerGroup(frontLeftDrive, leftColson, backLeftDrive),
            new SpeedControllerGroup(frontRightDrive, rightColson, backRightDrive)
        );

        this.ahrs = new AHRS(SPI.Port.kMXP);
        this.ahrs.reset();
    }

    @Override
    public void drive(double forwardRate, double strafeRate, double turnRate) {
        double angle = -this.ahrs.getYaw();
        SmartDashboard.putNumber("Gyro:", angle);

        if (this.getCenterWheelsDeployed()) {
            this.tankDrive.arcadeDrive(forwardRate, turnRate);
        } else {
            this.holoDrive.driveCartesian(strafeRate, forwardRate, turnRate, angle);
        }
    }

    @Override
    public void setCenterWheelsDeployed(boolean deployed) {
        this.leftColson.setDeployed(deployed);
        this.rightColson.setDeployed(deployed);

        this.frontLeftDrive.setScaledDown(deployed);
        this.frontRightDrive.setScaledDown(deployed);
        this.backLeftDrive.setScaledDown(deployed);
        this.backRightDrive.setScaledDown(deployed);
    }

    @Override
    public boolean getCenterWheelsDeployed() {
        // all pods should be synchronized so just pick one arbitrarily
        return this.leftColson.isDeployed();
    }

    @Override
    public void resetGyro() {
        this.ahrs.reset();
    }
}

