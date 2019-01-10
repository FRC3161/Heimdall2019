package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.Talon;

//TODO: Change it from the wpi talons to the CTRE

public class Drive {
    private MecanumDrive drivetrain;
    private Talon frontLeftDrive;
    private Talon frontRightDrive;
    private Talon backLeftDrive;
    private Talon backRightDrive;

    public void driveInit() {
        this.frontLeftDrive = new Talon(RobotMap.DRIVETRAIN_LEFT_FRONT_TALON);
        this.frontRightDrive = new Talon(RobotMap.DRIVETRAIN_LEFT_BACK_TALON);
        this.backLeftDrive = new Talon(RobotMap.DRIVETRAIN_RIGHT_FRONT_TALON);
        this.backRightDrive = new Talon(RobotMap.DRIVETRAIN_RIGHT_BACK_TALON);

        this.drivetrain = new MecanumDrive(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);
    }

    public void driveRobot(double forwardRate, double strafeRate, double turnRate) {
        this.drivetrain.driveCartesian(forwardRate, strafeRate, turnRate);
    }

}

