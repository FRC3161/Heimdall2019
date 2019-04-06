package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;

import com.kauailabs.navx.frc.AHRS;

import ca.team3161.lib.utils.Utils;
import frc.robot.InvertiblePIDSource;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class StarDriveImpl implements StarDrive {

    private static final double MANUAL_TURNING_DEADBAND = 0.05;
    private static final long UPDATE_WINDOW = 100;

    protected final MecanumDrive driveBase;
    protected final OmniPod frontLeftDrive;
    protected final OmniPod frontRightDrive;
    protected final OmniPod backLeftDrive;
    protected final OmniPod backRightDrive;
    protected final ColsonPod leftColson;
    protected final ColsonPod rightColson;

    protected InvertiblePIDSource<AHRS> angleSensor;

    protected final PIDController turnController;
    protected boolean fieldCentric = true;
    protected boolean speedLimited = false;
    protected double angleTarget;
    protected volatile double computedTurnPID;
    //ramps amount of output
    protected final double kP = 0.0106;
    //builds up over time and resets when target is hit
    protected final double kI = 0.0;
    //gets larger as the speed increases
    protected final double kD = 0.008;
    protected float kToleranceDegrees = 2;
    protected long lastUpdate = -1;

    public StarDriveImpl() {
        this.frontLeftDrive = Utils.safeInit("frontLeftDrive", () -> new RawOmniPodImpl(RobotMap.DRIVETRAIN_LEFT_FRONT_TALON));
        frontLeftDrive.setInverted(true);

        this.frontRightDrive = Utils.safeInit("frontRightDrive", () -> new RawOmniPodImpl(RobotMap.DRIVETRAIN_RIGHT_FRONT_TALON));
        frontRightDrive.setInverted(true);

        this.backLeftDrive = Utils.safeInit("backLeftDrive", () -> new RawOmniPodImpl(RobotMap.DRIVETRAIN_LEFT_BACK_TALON));
        backLeftDrive.setInverted(true);

        this.backRightDrive = Utils.safeInit("backRightDrive", () -> new RawOmniPodImpl(RobotMap.DRIVETRAIN_RIGHT_BACK_TALON));
        backRightDrive.setInverted(true);

        Solenoid colsonValve = Utils.safeInit("colsonValve", () -> new Solenoid(RobotMap.COLSON_SOLENOID));
        this.leftColson = Utils.safeInit("leftColson", () -> new ColsonPodImpl(RobotMap.DRIVETRAIN_LEFT_COLSON, colsonValve));
        this.leftColson.setInverted(true);
        this.rightColson = Utils.safeInit("rightColson", () -> new ColsonPodImpl(RobotMap.DRIVETRAIN_RIGHT_COLSON, colsonValve));
        this.rightColson.setInverted(true);

        this.driveBase = new MecanumDrive(new SpeedControllerGroup(frontLeftDrive, leftColson), backLeftDrive,
                frontRightDrive, new SpeedControllerGroup(rightColson, backRightDrive));

        this.angleSensor = new InvertiblePIDSource<>(new AHRS(SPI.Port.kMXP), AHRS::pidGet);
        this.angleSensor.setInverted(false);

        this.turnController = new PIDController(kP, kI, kD, angleSensor, this::gyroPID);
        turnController.setInputRange(-180, 180);
        turnController.setContinuous();
        turnController.setAbsoluteTolerance(kToleranceDegrees);
        turnController.enable();

        setAngleTarget(0);
    }

    @Override
    public void drive(double forwardRate, double strafeRate, double turnRate) {
        if (this.speedLimited) {
            final double limitFactor = 0.45;
            forwardRate *= limitFactor;
            strafeRate *= limitFactor;
            turnRate *= limitFactor;
        }

        if (getCenterWheelsDeployed()) {
            driveTank(forwardRate, turnRate);
        } else {
            driveOmni(forwardRate, strafeRate, turnRate);
        }
    }

    private void driveOmni(double forwardRate, double strafeRate, double turnRate) {
        double currentAngle = this.angleSensor.pidGet();
        SmartDashboard.putNumber("Gyro:", currentAngle);

        final double effectiveTurnRate;
        if (Math.abs(turnRate) < MANUAL_TURNING_DEADBAND) {
            effectiveTurnRate = computedTurnPID;
        } else {
            if (this.turnController.isEnabled()) {
                this.turnController.disable();
            }
            effectiveTurnRate = turnRate;
        }

        this.driveBase.driveCartesian(forwardRate, strafeRate, effectiveTurnRate, fieldCentric ? currentAngle : 0);
    }

    private void driveTank(double forwardRate, double turnRate) {
        this.driveBase.driveCartesian(forwardRate, 0, turnRate);
    }

    @Override
    public void setAngleTarget(double angleTarget) {
        long now = System.currentTimeMillis();
        if (lastUpdate + UPDATE_WINDOW > now) {
            return;
        }
        lastUpdate = now;
        this.angleTarget = angleTarget % 360; // mod 360 to handle wraparound
        turnController.setSetpoint(angleTarget);
        SmartDashboard.putNumber("Angle Target", angleTarget);
        if (!this.turnController.isEnabled()) {
            this.turnController.enable();
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
        this.angleSensor.getDevice().reset();
    }

    @Override
    public void setFieldCentric(boolean fieldCentric) {
        this.fieldCentric = fieldCentric;
        this.speedLimited = !fieldCentric;
    }

    @Override
    public boolean isFieldCentric() {
        return this.fieldCentric;
    }

    public void resetTurnController() {
        this.turnController.reset();
        this.angleTarget = this.angleSensor.pidGet();
    }

    //Sets the gyro to make the robot face a cetain angle
	protected void gyroPID(double angle) {
        this.computedTurnPID = angle;
        SmartDashboard.putNumber("pid value", angle);
    }
}
