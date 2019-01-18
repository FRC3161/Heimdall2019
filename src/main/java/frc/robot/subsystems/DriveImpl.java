package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;
import frc.robot.RobotMap;
import frc.robot.subsystems.TalonPIDSource;
import java.lang.Math;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveImpl implements Drive {
    private final MecanumDrive drivetrain;
    private final WPI_TalonSRX frontLeftDrive;
    private final WPI_TalonSRX frontRightDrive;
    private final WPI_TalonSRX backLeftDrive;
    private final WPI_TalonSRX backRightDrive;

    //PID
    double Kp;
    double Ki;
    double Kd;
    double Kf;
    PIDController pidcontrol;

    //For gyro
    private AHRS ahrs;
    private double angle;
    
    private double currentRotationRate;

    public DriveImpl() {
        this.frontLeftDrive = new WPI_TalonSRX(RobotMap.DRIVETRAIN_LEFT_FRONT_TALON);
        this.frontRightDrive = new WPI_TalonSRX(RobotMap.DRIVETRAIN_LEFT_BACK_TALON);
        this.backLeftDrive = new WPI_TalonSRX(RobotMap.DRIVETRAIN_RIGHT_FRONT_TALON);
        this.backRightDrive = new WPI_TalonSRX(RobotMap.DRIVETRAIN_RIGHT_BACK_TALON);

        this.drivetrain = new MecanumDrive(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);

        this.ahrs = new AHRS(SPI.Port.kMXP);
        this.ahrs.reset();

        //Setting PID Values so they aren't empty
        this.Kp = 0.001;
        this.Ki = 0.001;
        this.Kd = 0.001;

        //this.pidcontrol = new PIDController(Kp, Ki, Kd, ahrs, );
    }

    @Override
    public void drive(double forwardRate, double strafeRate, double turnRate) {
        //Stick deadzone
        if (Math.abs(forwardRate) <= 0.05) {
            forwardRate = 0;
        }

        if (Math.abs(strafeRate) <= 0.05) {
            strafeRate = 0;
        }

        if (Math.abs(turnRate) <= 0.05) {
            turnRate = 0;
        }
        double angle = -this.ahrs.getYaw();
        SmartDashboard.putNumber("Gyro:", angle);
        
        this.drivetrain.driveCartesian(forwardRate, strafeRate, turnRate, angle);
    }

    @Override
    public void setCenterWheelsDeployed(boolean deployed) {
        // TODO
    }

    @Override
    public boolean getCenterWheelsDeployed() {
        return false;
    }

    @Override
    public void resetGyro() {
        this.ahrs.reset();
    }
}

