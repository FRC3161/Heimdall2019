package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.Talon;

//TODO: Change it from the wpi talons to the CTRE

public class Drive{
    private MecanumDrive drivetrain;
    private Talon frontLeftDrive;
    private Talon frontRightDrive;
    private Talon backLeftDrive;
    private Talon backRightDrive;

    RobotMap obj = new RobotMap();



    public void driveInit() {


        frontLeftDrive = new Talon(RobotMap.DRIVETRAIN_LEFT_FRONT_TALON);
        frontRightDrive = new Talon(RobotMap.DRIVETRAIN_LEFT_BACK_TALON);
        backLeftDrive = new Talon(RobotMap.DRIVETRAIN_RIGHT_FRONT_TALON);
        backRightDrive = new Talon(RobotMap.DRIVETRAIN_RIGHT_BACK_TALON);


        drivetrain = new MecanumDrive(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);
    }

    public void driveRobot(double leftStickX, double leftStickY, double rightStickX) {
        drivetrain.driveCartesian(leftStickX, leftStickY, rightStickX);
    }

}

