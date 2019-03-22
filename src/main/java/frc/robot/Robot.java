/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import ca.team3161.lib.robot.TitanBot;
import ca.team3161.lib.utils.Utils;
import ca.team3161.lib.utils.controls.DeadbandJoystickMode;
import ca.team3161.lib.utils.controls.InvertedJoystickMode;
import ca.team3161.lib.utils.controls.LogitechDualAction;
import ca.team3161.lib.utils.controls.SquaredJoystickMode;
import ca.team3161.lib.utils.controls.Gamepad.PressType;
import ca.team3161.lib.utils.controls.LogitechDualAction.LogitechAxis;
import ca.team3161.lib.utils.controls.LogitechDualAction.LogitechButton;
import ca.team3161.lib.utils.controls.LogitechDualAction.LogitechControl;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import frc.robot.subsystems.drivetrain.Drive;
import frc.robot.subsystems.drivetrain.DriveImpl;
import frc.robot.subsystems.drivetrain.ManualTurningDriveImpl;
import frc.robot.subsystems.tower.Tower;
import frc.robot.subsystems.tower.TowerImpl;
import frc.robot.subsystems.tower.Tower.Direction;
import frc.robot.subsystems.tower.Tower.Position;
import frc.robot.subsystems.tower.GamePieceWatcher;

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
  private static final double GAMEPAD_DEADBAND = 0.05;
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private Compressor compressor;
  private Relay underLay;
  private Drive drive;
  private Tower tower;

  private LogitechDualAction driverPad;
  private LogitechDualAction operatorPad;
  String DSPrint;
  private static final long UPDATE_WINDOW = 3000;
  private long lastYeet = -1;

  @Override
  public int getAutonomousPeriodLengthSeconds() {
    return 15;
  }

  /**
   * This function is called once when the robot is disabled.
   */
  @Override
  public void disabledSetup() {
    this.drive.resetTurnController();
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
    this.driverPad = Utils.safeInit("driverPad", () -> new LogitechDualAction(0));
    this.operatorPad = Utils.safeInit("operatorPad", () -> new LogitechDualAction(1));
    this.driverPad.setMode(LogitechControl.LEFT_STICK, LogitechAxis.Y,
        new InvertedJoystickMode().andThen(new SquaredJoystickMode()).andThen(new DeadbandJoystickMode(GAMEPAD_DEADBAND)));
    this.driverPad.setMode(LogitechControl.LEFT_STICK, LogitechAxis.X,
        new SquaredJoystickMode().andThen(new DeadbandJoystickMode(GAMEPAD_DEADBAND)));
    this.driverPad.setMode(LogitechControl.RIGHT_STICK, LogitechAxis.X,
        new SquaredJoystickMode().andThen(new DeadbandJoystickMode(GAMEPAD_DEADBAND)));
    this.operatorPad.setMode(LogitechControl.LEFT_STICK, LogitechAxis.Y,
      new DeadbandJoystickMode(GAMEPAD_DEADBAND));
    this.operatorPad.setMode(LogitechControl.RIGHT_STICK, LogitechAxis.Y,
      new DeadbandJoystickMode(GAMEPAD_DEADBAND));
    this.compressor = Utils.safeInit("compressor", () -> new Compressor());
    this.compressor.setClosedLoopControl(true);
    this.drive = Utils.safeInit("drive", () -> new ManualTurningDriveImpl());
    this.tower = Utils.safeInit("tower", () -> new TowerImpl());
    this.underLay = Utils.safeInit("underLay", () -> new Relay(RobotMap.UNDERGLOW_SPIKE, Relay.Direction.kForward));
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    m_chooser.addOption("System Check Auto", kSystemCheckAuto);
    SmartDashboard.putData("Auto choices pls", m_chooser);

    registerLifecycleComponent(this.driverPad);
    registerLifecycleComponent(this.operatorPad);
    registerLifecycleComponent(this.tower);
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

    // Do teleop stuff and enable gamepads
    this.teleopSetup();
    driverPad.enableBindings();
    operatorPad.enableBindings();
  }

  /**
   * This function is called at the start of autonomous and runs top-down in a
   * "script-like" manner
   */
  @Override
  public void autonomousRoutine() throws InterruptedException {
    // Just do teleop, at usual 50Hz
    while (isAutonomous()) {
      this.teleopRoutine();
      Thread.sleep((long) (TimedRobot.kDefaultPeriod * 1000));
    }
  }

  /**
   * This function is called once at the start of operator control.
   */
  @Override
  public void teleopSetup() {
    this.driverPad.bind(LogitechButton.RIGHT_TRIGGER, PressType.PRESS, this.drive::toggleCenterWheelsDeployed);
    this.driverPad.bind(LogitechButton.RIGHT_BUMPER, x -> this.drive.setFieldCentric(!x));
    this.driverPad.bind(LogitechButton.START, this.drive::resetGyro);
    this.operatorPad.bind(LogitechButton.START, this.tower::reset);
    this.operatorPad.bind(LogitechButton.B, PressType.PRESS, () -> this.tower.setBeakOpen(false));
    this.operatorPad.bind(LogitechButton.B, PressType.RELEASE,() -> this.tower.setBeakOpen(true));
    this.operatorPad.bind(LogitechButton.RIGHT_BUMPER, PressType.PRESS, this::wristUp);
    this.operatorPad.bind(LogitechButton.RIGHT_BUMPER, PressType.RELEASE, this::wristStop);
    this.operatorPad.bind(LogitechButton.LEFT_BUMPER, PressType.PRESS, this::wristDown);
    this.operatorPad.bind(LogitechButton.LEFT_BUMPER, PressType.RELEASE, this::wristStop);
    this.operatorPad.map(LogitechControl.RIGHT_STICK, LogitechAxis.Y, x-> {
      if (x != 0 && operatorPad.getDpad() == 270) {
        this.tower.setElevatorSpeed(x);
      }
    });
    this.operatorPad.map(LogitechControl.LEFT_STICK, LogitechAxis.Y, x-> {
      if (x!= 0 && operatorPad.getDpad() == 270) {
        this.tower.setArmSpeed(x);
      }
    });
    this.underLay.set(Value.kOn);

    this.driverPad.enableBindings();
    this.operatorPad.enableBindings();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopRoutine() {
    SmartDashboard.putNumber("operator pad left stick Y", operatorPad.getValue(LogitechControl.LEFT_STICK, LogitechAxis.Y));
    SmartDashboard.putNumber("operator pad right stick Y", operatorPad.getValue(LogitechControl.RIGHT_STICK, LogitechAxis.Y));
    SmartDashboard.putBoolean("operator pad A button", operatorPad.getButton(LogitechButton.A));
    SmartDashboard.putBoolean("operator pad B button", operatorPad.getButton(LogitechButton.B));
    SmartDashboard.putBoolean("operator pad right trigger", operatorPad.getButton(LogitechButton.RIGHT_TRIGGER));
    SmartDashboard.putBoolean("Colson Deployment", drive.getCenterWheelsDeployed());
    SmartDashboard.putBoolean("is the compressor on", this.compressor.getClosedLoopControl());
    SmartDashboard.putNumber("Dpad angle", operatorPad.getDpad());
    tower.putEncoderTicks();
    this.drive.drive(
      this.driverPad.getValue(LogitechControl.LEFT_STICK, LogitechAxis.Y),
      this.driverPad.getValue(LogitechControl.LEFT_STICK, LogitechAxis.X),
      this.driverPad.getValue(LogitechControl.RIGHT_STICK, LogitechAxis.X)
    );
    Double faceTarget = null;
    // Look Forward
    if (driverPad.getButton(LogitechButton.Y)) {
      faceTarget = 0.0;
    }
    // Look Right
    if (driverPad.getButton(LogitechButton.B)) {
      faceTarget = 90.0;
    }
    // Look Backward
    if (driverPad.getButton(LogitechButton.A)) {
      faceTarget = 180.0;
    }
    // Look Left
    if (driverPad.getButton(LogitechButton.X)) {
      faceTarget = -90.0;
    }
    //combo directions
    if (driverPad.getButton(LogitechButton.Y) && driverPad.getButton(LogitechButton.B)) {
      faceTarget = 28.75;
    }
    if (driverPad.getButton(LogitechButton.Y) && driverPad.getButton(LogitechButton.X)) {
      faceTarget = -28.75;
    }
    if (driverPad.getButton(LogitechButton.A) && driverPad.getButton(LogitechButton.B)) {
      faceTarget = 151.25;
    }
    if (driverPad.getButton(LogitechButton.A) && driverPad.getButton(LogitechButton.X)) {
      faceTarget = -151.25;
    }
    if (faceTarget != null) {
      drive.setAngleTarget(faceTarget);
      SmartDashboard.putNumber("Face Target", faceTarget);
    }

    // TODO properly map out all tower positions
    if (operatorPad.getDpad() == 180){
      tower.setTowerPosition(Position.GROUND);
    }
    if (operatorPad.getDpad() == 0){
      toggleCompressor();
    } 

    if (operatorPad.getButton(LogitechButton.A)) {
      tower.setTowerPosition(Position.LEVEL_1);
    }
    if (operatorPad.getButton(LogitechButton.X)) {
      tower.setTowerPosition(Position.LEVEL_2);
    }

    if (operatorPad.getButton(LogitechButton.Y)) {
      tower.setTowerPosition(Position.LEVEL_3);
    }

    //TODO hatch, rollers
    if ((!operatorPad.getButton(LogitechButton.LEFT_TRIGGER))&& (!operatorPad.getButton(LogitechButton.RIGHT_TRIGGER))) {
      tower.setRollers(Direction.NONE);
    }
    if (operatorPad.getButton(LogitechButton.LEFT_TRIGGER)) { //Dpad UP
      tower.setRollers(Direction.OUT);
      this.easterEgg();
    }
    if (operatorPad.getButton(LogitechButton.RIGHT_TRIGGER) ) { //Dpad DOWN
      tower.setRollers(Direction.IN);
    }
  }

  public void easterEgg(){
    long now = System.currentTimeMillis();
    if (lastYeet + UPDATE_WINDOW > now) {
        return;
    }
    DriverStation.reportError("Yeet", false);
    lastYeet = now;
  }

  private void toggleCompressor() {
    this.compressor.setClosedLoopControl(!this.compressor.getClosedLoopControl());
  }

  private void wristUp() {
    if (tower.isBeakOpen()) {
      wristStop();
      return;
    }
    this.tower.setWristSpeed(1);
  }

  private void wristDown() {
    if (tower.isBeakOpen()) {
      wristStop();
      return;
    }
    this.tower.setWristSpeed(-1);
  }

  private void wristStop() {
      this.tower.setWristSpeed(0);
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
