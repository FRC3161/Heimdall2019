package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.PIDController;
import com.kauailabs.navx.frc.AHRS;
import frc.robot.RobotMap;
import java.lang.Math;

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

    public DriveImpl() {
        this.frontLeftDrive = new WPI_TalonSRX(RobotMap.DRIVETRAIN_LEFT_FRONT_TALON);
        this.frontRightDrive = new WPI_TalonSRX(RobotMap.DRIVETRAIN_LEFT_BACK_TALON);
        this.backLeftDrive = new WPI_TalonSRX(RobotMap.DRIVETRAIN_RIGHT_FRONT_TALON);
        this.backRightDrive = new WPI_TalonSRX(RobotMap.DRIVETRAIN_RIGHT_BACK_TALON);

        this.drivetrain = new MecanumDrive(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);

        this.pidcontrol = new PIDController(Kp, Ki, Kd, frc.robot.Robot.ahrs, this::gyroPIDWrite);
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

        if (Math.abs(forwardRate) <= 0.05) {
            turnRate = 0;
        }  
        
        // TODO make this field-centric?
        this.drivetrain.driveCartesian(forwardRate, strafeRate, turnRate);
    }

    @Override
    public void setCenterWheelsDeployed(boolean deployed) {
        // TODO
    }

    @Override
    public boolean getCenterWheelsDeployed() {
        return false;
    }
}

