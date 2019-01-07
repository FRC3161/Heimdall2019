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

public class robotcode2019 {
	//This is declaring the motors with the corresponding Controllers
		private WPI_TalonSRX frontLeftDrive;
		private WPI_TalonSRX frontRightDrive;
		private WPI_TalonSRX backLeftDrive;
		private WPI_TalonSRX backRightDrive;
		private VictorSP IntakeL = new VictorSP (3);
		private VictorSP IntakeR = new VictorSP (2);
		private VictorSP pivot = new VictorSP (1);
		private WPI_TalonSRX leftElevator = new WPI_TalonSRX (4);
		private VictorSP leftElevatorSlave = new VictorSP(0);
		private WPI_TalonSRX rightElevator = new WPI_TalonSRX (6);
		private DigitalInput topLimitSwitch = new DigitalInput(0);
		private DigitalInput bottomLimitSwitch = new DigitalInput(1);

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
		double rightStickY;
		boolean getButtonSTART;
		boolean getButtonY;
		boolean getButtonX;
		boolean getButtonA;
		boolean getButtonB;
		boolean getButtonRT;

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
		resetWheelEncoders();
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
		getButtonSTART = driverPad.getButton(LogitechDualAction.LogitechButton.START);
		getButtonY = driverPad.getButton(LogitechDualAction.LogitechButton.Y);
		getButtonX = driverPad.getButton(LogitechDualAction.LogitechButton.X);
		getButtonA = driverPad.getButton(LogitechDualAction.LogitechButton.A);
		getButtonRT = driverPad.getButton(LogitechDualAction.LogitechButton.RIGHT_TRIGGER);
		getButtonB = driverPad.getButton(LogitechDualAction.LogitechButton.B);
		getButtonRB = driverPad.getButton(LogitechDualAction.LogitechButton.RIGHT_BUMPER);
		
		
		
			
		
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

		//Preset angles for the robot - to be called with buttons A, B, X, Y
		rotateToAngle = false;
		if (getButtonSTART) 
		{
			ahrs.reset();
		}
		//Look Forward
				if (getButtonY) 
				{
					currentRotationRate = gyroPID(0.0);
					rotateToAngle = true;
				}
				//Look Right
				else if (getButtonB) 
				{
					currentRotationRate = gyroPID(90.0);
					rotateToAngle = true;
				}
				//Look Backward
				else if (getButtonA) 
				{
					currentRotationRate = gyroPID(180.0);
					rotateToAngle = true;
				}
				//Look Left
				else if (getButtonX) 
				{
					currentRotationRate = gyroPID(-90.0);
					rotateToAngle = true;
				}

				if(!rotateToAngle)
				{
					turnController.disable();
					currentRotationRate = rightStickX;
				}
				
	}
	}
