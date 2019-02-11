/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.concurrent.TimeUnit;

import ca.team3161.lib.robot.TitanBot;
import ca.team3161.lib.utils.controls.DeadbandJoystickMode;
import ca.team3161.lib.utils.controls.InvertedJoystickMode;
import ca.team3161.lib.utils.controls.LogitechDualAction;
import ca.team3161.lib.utils.controls.SquaredJoystickMode;
import ca.team3161.lib.utils.controls.Gamepad.PressType;
import ca.team3161.lib.utils.controls.LogitechDualAction.LogitechAxis;
import ca.team3161.lib.utils.controls.LogitechDualAction.LogitechButton;
import ca.team3161.lib.utils.controls.LogitechDualAction.LogitechControl;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import frc.robot.subsystems.drivetrain.Drive;
import frc.robot.subsystems.drivetrain.DriveImpl;
import frc.robot.subsystems.tower.Tower;
import frc.robot.subsystems.tower.TowerImpl;
import frc.robot.subsystems.tower.Tower.Direction;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TitanBot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private static final String kSystemCheckAuto = "System Check";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private Compressor compressor;
  private Relay underLay;
  private Drive drive;
  private Tower tower;

  private LogitechDualAction driverPad;
  private LogitechDualAction operatorPad;

  @Override
  public int getAutonomousPeriodLengthSeconds() {
    return 15;
  }

  /**
   * This function is called once when the robot is disabled.
   */
  @Override
  public void disabledSetup() {
  }

  /**
   * This function is called periodically while the robot is disabled.
   */
  @Override
  public void disabledRoutine() {
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotSetup() {
    this.driverPad = new LogitechDualAction(0);
    this.operatorPad = new LogitechDualAction(1);
    this.driverPad.setMode(LogitechControl.LEFT_STICK, LogitechAxis.Y,
        new InvertedJoystickMode().andThen(new SquaredJoystickMode()).andThen(new DeadbandJoystickMode(0.05)));
    this.driverPad.setMode(LogitechControl.LEFT_STICK, LogitechAxis.X,
        new SquaredJoystickMode().andThen(new DeadbandJoystickMode(0.05)));
    this.driverPad.setMode(LogitechControl.RIGHT_STICK, LogitechAxis.X,
        new SquaredJoystickMode().andThen(new DeadbandJoystickMode(0.05)));
    this.compressor = new Compressor();
    this.compressor.setClosedLoopControl(true);
    this.drive = new DriveImpl();
    this.tower = new TowerImpl();
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    m_chooser.addOption("System Check Auto", kSystemCheckAuto);
    SmartDashboard.putData("Auto choices pls", m_chooser);

    registerLifecycleComponent(this.driverPad);
    registerLifecycleComponent(this.operatorPad);
    registerLifecycleComponent(this.tower);
    this.underLay.setDirection(Relay.Direction.kForward);
    CameraServer.getInstance().startAutomaticCapture();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString line to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure below with additional strings. If using the SendableChooser
   * make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousSetup() {
    m_autoSelected = m_chooser.getSelected();
    // autoSelected = SmartDashboard.getString("Auto Selector",
    // defaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    drive.resetGyro();
    this.underLay.set(Value.kOn);
  }

  /**
   * This function is called at the start of autonomous and runs top-down in a
   * "script-like" manner
   */
  @Override
  public void autonomousRoutine() throws InterruptedException {
    switch (m_autoSelected) {
    case kCustomAuto:
      // Put custom auto code here
      break;

    case kSystemCheckAuto:
      tower.setBeakOpen(true);
      tower.setClawOpen(true);
      tower.setRollers(Direction.IN);
      waitFor(250, TimeUnit.MILLISECONDS);
      tower.setBeakOpen(false);
      tower.setClawOpen(false);
      tower.setRollers(Direction.NONE);
      waitFor(1, TimeUnit.SECONDS);
      drive.setCenterWheelsDeployed(true);
      waitFor(250, TimeUnit.MILLISECONDS);
      drive.setCenterWheelsDeployed(false);
      break;

    case kDefaultAuto:
    default:
      // Put default auto code here
      break;
    }
  }

  /**
   * This function is called once at the start of operator control.
   */
  @Override
  public void teleopSetup() {
    this.driverPad.bind(LogitechButton.RIGHT_TRIGGER, PressType.PRESS,
        () -> this.drive.setCenterWheelsDeployed(!this.drive.getCenterWheelsDeployed()));
    this.driverPad.bind(LogitechButton.RIGHT_BUMPER, x -> this.drive.setFieldCentric(!x));
    this.driverPad.bind(LogitechButton.START, this.drive::resetGyro);
    this.operatorPad.map(LogitechControl.LEFT_STICK, LogitechAxis.Y, this.tower::setElevatorSpeed);
    this.operatorPad.map(LogitechControl.RIGHT_STICK, LogitechAxis.Y, this.tower::setArmSpeed);
    this.underLay.set(Value.kOn);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopRoutine() {
    this.drive.drive(
      this.driverPad.getValue(LogitechControl.LEFT_STICK, LogitechAxis.Y),
      this.driverPad.getValue(LogitechControl.LEFT_STICK, LogitechAxis.X),
      this.driverPad.getValue(LogitechControl.RIGHT_STICK, LogitechAxis.X)
    );
    if (driverPad.getButton(LogitechButton.Y)) {
      drive.setAngleTarget(0.0);
    }
    // Look Right
    else if (driverPad.getButton(LogitechButton.B)) {
      drive.setAngleTarget(90.0);
    }
    // Look Backward
    else if (driverPad.getButton(LogitechButton.A)) {
      drive.setAngleTarget(180.0);
    }
    // Look Left
    else if (driverPad.getButton(LogitechButton.X)) {
      drive.setAngleTarget(-90.0);
    } else if (driverPad.getButton(LogitechButton.Y) && driverPad.getButton(LogitechButton.B)) {
      drive.setAngleTarget(45.0);
    } else if (driverPad.getButton(LogitechButton.Y) && driverPad.getButton(LogitechButton.X)) {
      drive.setAngleTarget(-45.0);
    } else if (driverPad.getButton(LogitechButton.A) && driverPad.getButton(LogitechButton.B)) {
      drive.setAngleTarget(135.0);
    } else if (driverPad.getButton(LogitechButton.A) && driverPad.getButton(LogitechButton.X)) {
      drive.setAngleTarget(-135.0);
    }
  }

  /**
   * This function is called once at the start of test mode.
   */
  @Override
  public void testSetup() {
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testRoutine() {
  }
}
