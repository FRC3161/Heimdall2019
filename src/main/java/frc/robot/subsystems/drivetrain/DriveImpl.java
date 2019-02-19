package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;

import com.kauailabs.navx.frc.AHRS;

import ca.team3161.lib.robot.motion.drivetrains.SpeedControllerGroup;
import frc.robot.InvertiblePIDSource;
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

    private InvertiblePIDSource<AHRS> angleSensor;

    private final PIDController turnController;
    private boolean fieldCentric = true;
    private double angleTarget;
    private volatile double computedTurnPID;
    //ramps amount of output
    private final double kP = 0.005;
    //builds up over time and resets when target is hit
    private final double kI = 0.0;
    //gets larger as the speed increases
    private final double kD = 0.0;
    float kToleranceDegrees = 2;


    public DriveImpl() {
        this.frontLeftDrive = new RawOmniPodImpl(RobotMap.DRIVETRAIN_LEFT_FRONT_TALON);
        frontLeftDrive.setInverted(true);
        this.frontRightDrive = new RawOmniPodImpl(RobotMap.DRIVETRAIN_RIGHT_FRONT_TALON);
        frontRightDrive.setInverted(false);
        this.backLeftDrive = new RawOmniPodImpl(RobotMap.DRIVETRAIN_LEFT_BACK_TALON);
        backLeftDrive.setInverted(false);
        this.backRightDrive = new RawOmniPodImpl(RobotMap.DRIVETRAIN_RIGHT_BACK_TALON);
        backRightDrive.setInverted(true);

        Solenoid colsonValve = new Solenoid(RobotMap.COLSON_SOLENOID);
        this.leftColson = new ColsonPodImpl(RobotMap.DRIVETRAIN_LEFT_COLSON, colsonValve);
        this.rightColson = new ColsonPodImpl(RobotMap.DRIVETRAIN_RIGHT_COLSON, colsonValve);

        this.holoDrive = new MecanumDrive(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);
        this.tankDrive = new DifferentialDrive(
            new SpeedControllerGroup(frontRightDrive, leftColson, frontLeftDrive),
            new SpeedControllerGroup(backLeftDrive, rightColson, backRightDrive)
        );
        this.tankDrive.setSafetyEnabled(false);
        this.holoDrive.setSafetyEnabled(false);

        this.angleSensor = new InvertiblePIDSource<>(new AHRS(SPI.Port.kMXP), AHRS::getAngle);
        this.angleSensor.setInverted(true);

        this.turnController = new PIDController(kP, kI, kD, angleSensor, this::gyroPID);
        turnController.setInputRange(0, 360.0);
        turnController.setContinuous();
        turnController.setAbsoluteTolerance(kToleranceDegrees);
        turnController.enable();

        setAngleTarget(0);
    }

    @Override
    public void drive(double forwardRate, double strafeRate, double turnRate) {
        double currentAngle = this.angleSensor.pidGet();
        SmartDashboard.putNumber("Gyro:", currentAngle);

        if (this.getCenterWheelsDeployed()) {
            this.tankDrive.arcadeDrive(forwardRate, turnRate);
        } else {
	    if (!this.turnController.isEnabled()) {
		    this.turnController.enable();
	    }
            this.setAngleTarget(this.angleTarget + turnRate * 180 * TimedRobot.kDefaultPeriod); // 180 degrees per second, divided by update rate
            this.holoDrive.driveCartesian(strafeRate, forwardRate, computedTurnPID, fieldCentric ? currentAngle : 0);
        }
    }

    @Override
    public void setAngleTarget(double angleTarget) {
        if (angleTarget < 0) {
            angleTarget += 360;
        }
        this.angleTarget = angleTarget % 360; // mod 360 to handle wraparound
        turnController.setSetpoint(angleTarget);
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
        this.angleSensor.getDevice().reset();
    }

    @Override
    public void setFieldCentric(boolean fieldCentric) {
        this.fieldCentric = fieldCentric;
    }

    public void resetTurnController() {
	this.turnController.reset();
    }

    //Sets the gyro to make the robot face a cetain angle
	private void gyroPID(double angle) {
        this.computedTurnPID = angle;
        SmartDashboard.putNumber("pid value", angle);
    }
}

