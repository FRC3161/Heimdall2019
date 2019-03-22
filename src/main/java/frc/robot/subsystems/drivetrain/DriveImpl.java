package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;

import com.kauailabs.navx.frc.AHRS;

import ca.team3161.lib.robot.motion.drivetrains.SpeedControllerGroup;
import ca.team3161.lib.utils.Utils;
import frc.robot.InvertiblePIDSource;
import frc.robot.MathUtils;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveImpl implements Drive {
    protected final MecanumDrive holoDrive;
    protected final DifferentialDrive tankDrive;
    protected final OmniPod frontLeftDrive;
    protected final OmniPod frontRightDrive;
    protected final OmniPod backLeftDrive;
    protected final OmniPod backRightDrive;
    protected final ColsonPod leftColson;
    protected final ColsonPod rightColson;

    protected InvertiblePIDSource<AHRS> angleSensor;

    protected final PIDController turnController;
    protected boolean fieldCentric = true;
    protected boolean speedLimited = true;
    protected double angleTarget;
    protected volatile double computedTurnPID;
    //ramps amount of output
    protected final double kP = 0.0106;
    //builds up over time and resets when target is hit
    protected final double kI = 0.0;
    //gets larger as the speed increases
    protected final double kD = 0.008;
    protected float kToleranceDegrees = 2;

    public DriveImpl() {
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
        this.rightColson = Utils.safeInit("rightColson", () -> new ColsonPodImpl(RobotMap.DRIVETRAIN_RIGHT_COLSON, colsonValve));

        this.holoDrive = new MecanumDrive(frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive);
        this.tankDrive = new DifferentialDrive(
            new SpeedControllerGroup(frontLeftDrive, leftColson, frontRightDrive),
            new SpeedControllerGroup(backLeftDrive, rightColson, backRightDrive)
        );
        this.tankDrive.setSafetyEnabled(false);
        this.holoDrive.setSafetyEnabled(false);

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
        double currentAngle = this.angleSensor.pidGet();
        SmartDashboard.putNumber("Gyro:", currentAngle);

        if (this.speedLimited) {
            final double limit = 0.30;
            forwardRate = MathUtils.absClamp(forwardRate, limit);
            strafeRate = MathUtils.absClamp(strafeRate, limit);
            turnRate = MathUtils.absClamp(turnRate, limit);
        }

        if (this.getCenterWheelsDeployed()) {
            this.tankDrive.arcadeDrive(forwardRate, turnRate);
        } else {
            if (!this.turnController.isEnabled()) {
                this.turnController.enable();
            }
            this.setAngleTarget(this.angleTarget + turnRate * 180 * TimedRobot.kDefaultPeriod); // 180 degrees per second, divided by update rate
            this.holoDrive.driveCartesian(forwardRate,strafeRate,computedTurnPID, fieldCentric ? currentAngle : 0);
        }
    }

    @Override
    public void setAngleTarget(double angleTarget) {
        // if (angleTarget < 0) {
        //     angleTarget += 360;
        // }
        this.angleTarget = angleTarget % 360; // mod 360 to handle wraparound
        turnController.setSetpoint(angleTarget);
        SmartDashboard.putNumber("Angle Target", angleTarget);
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
