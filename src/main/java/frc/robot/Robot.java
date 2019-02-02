/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import ca.team3161.lib.robot.sensors.RightSight;
import ca.team3161.lib.utils.controls.DeadbandJoystickMode;
import ca.team3161.lib.utils.controls.Gamepad;
import ca.team3161.lib.utils.controls.InvertedJoystickMode;
import ca.team3161.lib.utils.controls.LogitechDualAction;
import ca.team3161.lib.utils.controls.SquaredJoystickMode;
import ca.team3161.lib.utils.controls.LogitechDualAction.LogitechAxis;
import ca.team3161.lib.utils.controls.LogitechDualAction.LogitechControl;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import frc.robot.subsystems.drivetrain.Drive;
import frc.robot.subsystems.drivetrain.DriveImpl;
import frc.robot.subsystems.tower.Tower;
import frc.robot.subsystems.tower.TowerImpl;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
// TODO evaluate switching to TitanBot for autonomous "script" style
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private Drive drive;
  private Tower tower;

  private Gamepad driverPad;
  private RightSight rightSight;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    this.driverPad = new LogitechDualAction(0);
    this.drive = new DriveImpl();
    this.tower = new TowerImpl();
    this.rightSight = new RightSight(0);
    this.rightSight.setInverted(true);
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices pls", m_chooser);

    CameraServer.getInstance().startAutomaticCapture();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // autoSelected = SmartDashboard.getString("Auto Selector",
    // defaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    drive.resetGyro();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    this.driverPad.setMode(LogitechControl.LEFT_STICK, LogitechAxis.Y, new InvertedJoystickMode().andThen(new SquaredJoystickMode()).andThen(new DeadbandJoystickMode(0.05)));
    this.driverPad.setMode(LogitechControl.LEFT_STICK, LogitechAxis.X, new SquaredJoystickMode().andThen(new DeadbandJoystickMode(0.05)));
    this.driverPad.setMode(LogitechControl.RIGHT_STICK, LogitechAxis.X, new SquaredJoystickMode().andThen(new DeadbandJoystickMode(0.05)));
    this.drive.drive(
      this.driverPad.getValue(LogitechControl.LEFT_STICK, LogitechAxis.Y),
      this.driverPad.getValue(LogitechControl.LEFT_STICK, LogitechAxis.X),
      this.driverPad.getValue(LogitechControl.RIGHT_STICK, LogitechAxis.X)
    );
    if(this.driverPad.getButton(LogitechDualAction.LogitechButton.START))
    {
      this.drive.resetGyro();
    }
    SmartDashboard.putBoolean("rightsight", rightSight.get());
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
