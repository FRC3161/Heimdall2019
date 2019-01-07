package org.usfirst.frc.team3161.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import ca.team3161.lib.utils.controls.InvertedJoystickMode;
import ca.team3161.lib.utils.controls.LogitechDualAction;
import ca.team3161.lib.utils.controls.LogitechDualAction.LogitechAxis;
import ca.team3161.lib.utils.controls.LogitechDualAction.LogitechControl;
import ca.team3161.lib.utils.controls.SquaredJoystickMode;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotCode2019 {
	//This is declaring the motors with the corresponding Controllers
		private WPI_TalonSRX frontLeftDrive;
		private WPI_TalonSRX frontRightDrive;
		private WPI_TalonSRX backLeftDrive;
		private WPI_TalonSRX backRightDrive;


		//Variables to be used when waiting
		boolean isWaiting = false;
		long waitStartTime = -1;

		private int firstAction = 0;
		private int secondAction = 0;

		//Declaring the way the robot will drive - RoboDrive class
		private MecanumDrive drivetrain;

		//Declaring the AHRS class to get gyro headings
		private AHRS ahrs;
		private double angle;
		//Configures all the Joysticks and Buttons for both controllers
		double leftStickX;
		double leftStickY;	 
		double rightStickX;

	private LogitechDualAction driverPad = new LogitechDualAction(0);
	public void robotInit() 
	{	
		frontLeftDrive = new WPI_TalonSRX(0);
		backLeftDrive = new WPI_TalonSRX(2);
		frontRightDrive = new WPI_TalonSRX(1);
		backRightDrive = new WPI_TalonSRX(3);
		
		frontLeftDrive.setInverted(false);
		frontLeftDrive.configPeakCurrentLimit(DRIVETRAIN_PEAK_CURRENT_AMP_LIMIT, 10);
		frontLeftDrive.configPeakCurrentDuration(DRIVETRAIN_PEAK_CURRENT_DURATION, 10);
		frontLeftDrive.configContinuousCurrentLimit(DRIVETRAIN_CONTINUOUS_CURRENT_AMP_LIMIT, 10);
		frontLeftDrive.enableCurrentLimit(true);

		frontRightDrive.setInverted(false);
		frontRightDrive.configPeakCurrentLimit(DRIVETRAIN_PEAK_CURRENT_AMP_LIMIT, 10);
		frontRightDrive.configPeakCurrentDuration(DRIVETRAIN_PEAK_CURRENT_DURATION, 10);
		frontRightDrive.configContinuousCurrentLimit(DRIVETRAIN_CONTINUOUS_CURRENT_AMP_LIMIT, 10);
		frontRightDrive.enableCurrentLimit(true);

		backRightDrive.setInverted(false);
		backRightDrive.configPeakCurrentLimit(DRIVETRAIN_PEAK_CURRENT_AMP_LIMIT, 10);
		backRightDrive.configPeakCurrentDuration(DRIVETRAIN_PEAK_CURRENT_DURATION, 10);
		backRightDrive.configContinuousCurrentLimit(DRIVETRAIN_CONTINUOUS_CURRENT_AMP_LIMIT, 10);
		backRightDrive.enableCurrentLimit(true);

		backLeftDrive.setInverted(false);
		backLeftDrive.configPeakCurrentLimit(DRIVETRAIN_PEAK_CURRENT_AMP_LIMIT, 10);
		backLeftDrive.configPeakCurrentDuration(DRIVETRAIN_PEAK_CURRENT_DURATION, 10);
		backLeftDrive.configContinuousCurrentLimit(DRIVETRAIN_CONTINUOUS_CURRENT_AMP_LIMIT, 10);
		backLeftDrive.enableCurrentLimit(true);
		//gyro readings for kmxp
		ahrs = new AHRS(SPI.Port.kMXP);
		ahrs.reset();
		drivetrain = new MecanumDrive(frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive);
	}
	public void teleopInit()
	{
		drivetrain.setSafetyEnabled(true);
		FLController.disable();
		FRController.disable();
		BLController.disable();
		BRController.disable();
	}

	public void teleopPeriodic() 
	{

		showDisplay();

		leftStickX = driverPad.getValue(LogitechDualAction.LogitechControl.LEFT_STICK, LogitechDualAction.LogitechAxis.X);
		leftStickY = driverPad.getValue(LogitechDualAction.LogitechControl.LEFT_STICK, LogitechDualAction.LogitechAxis.Y);
		rightStickX = driverPad.getValue(LogitechDualAction.LogitechControl.RIGHT_STICK, LogitechDualAction.LogitechAxis.X);
		

		
		if (Math.abs(rightStickX) < 0.05)
		{
			rightStickX = 0;
		}
		if (Math.abs(leftStickY) < 0.05) 
		{
			leftStickY = 0;
		}
		if (Math.abs(leftStickX) < 0.05) 
		{
			leftStickX = 0;
		}
		//Calls upon the mecanumDrive_Cartesian method that sends specific power to the talons
		drivetrain.driveCartesian (leftStickX, leftStickY, rightStickX);
	}
