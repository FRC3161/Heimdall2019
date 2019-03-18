/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

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
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import frc.robot.subsystems.drivetrain.Drive;
import frc.robot.subsystems.drivetrain.DriveImpl;
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

  private GamePieceWatcher gamePieceWatcher;

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
    this.driverPad = new LogitechDualAction(0);
    this.operatorPad = new LogitechDualAction(1);
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
    this.compressor = new Compressor();
    this.compressor.setClosedLoopControl(true);
    this.drive = new DriveImpl();
    this.tower = new TowerImpl();
    this.underLay = new Relay(RobotMap.UNDERGLOW_SPIKE, Relay.Direction.kForward);
    this.gamePieceWatcher = new GamePieceWatcher();
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
    this.operatorPad.bind(LogitechButton.B, PressType.PRESS, () -> this.tower.setClawOpen(false));
    this.operatorPad.bind(LogitechButton.B, PressType.RELEASE,() -> this.tower.setClawOpen(true));
    this.operatorPad.bind(LogitechButton.X, PressType.PRESS, this::toggleCompressor);
    this.driverPad.bind(LogitechButton.RIGHT_BUMPER, x -> this.drive.setFieldCentric(!x));
    this.driverPad.bind(LogitechButton.START, this.drive::resetGyro);
    this.operatorPad.bind(LogitechButton.START, this.tower::reset);
    this.operatorPad.map(LogitechControl.RIGHT_STICK, LogitechAxis.Y, x-> {
      if (x!= 0) {
        this.tower.setElevatorSpeed(x);
      }
    });
    this.operatorPad.map(LogitechControl.LEFT_STICK, LogitechAxis.Y, x-> {
      if (x!= 0) {
        this.tower.setArmSpeed(x);
      }
    });
    this.operatorPad.bind(LogitechButton.A, PressType.PRESS, this.tower::toggleClaw);
    this.underLay.set(Value.kOn);
    this.drive.resetGyro();

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
    SmartDashboard.putBoolean("are colsons down", this.drive.getCenterWheelsDeployed());
    tower.putEncoderTicks();
    this.drive.drive(
      this.driverPad.getValue(LogitechControl.LEFT_STICK, LogitechAxis.Y),
      this.driverPad.getValue(LogitechControl.LEFT_STICK, LogitechAxis.X),
      this.driverPad.getValue(LogitechControl.RIGHT_STICK, LogitechAxis.X)
    );
    double faceTarget = 0;
    // Look Forward
    if (driverPad.getButton(LogitechButton.Y)) {
      faceTarget = 0;
    }
    // Look Right
    if (driverPad.getButton(LogitechButton.B)) {
      faceTarget = 270;
    }
    // Look Backward
    if (driverPad.getButton(LogitechButton.A)) {
      faceTarget = 180;
    }
    // Look Left
    if (driverPad.getButton(LogitechButton.X)) {
      faceTarget = 90;
    }
    //combo directions
    if (driverPad.getButton(LogitechButton.Y) && driverPad.getButton(LogitechButton.B)) {
      faceTarget = 315;
    }
    if (driverPad.getButton(LogitechButton.Y) && driverPad.getButton(LogitechButton.X)) {
      faceTarget = 45;
    }
    if (driverPad.getButton(LogitechButton.A) && driverPad.getButton(LogitechButton.B)) {
      faceTarget = 255;
    }
    if (driverPad.getButton(LogitechButton.A) && driverPad.getButton(LogitechButton.X)) {
      faceTarget = 135;
    }
    drive.setAngleTarget(faceTarget);

    // TODO properly map out all tower positions

    if (operatorPad.getDpad() == 270) {
      tower.setTowerPosition(Position.HATCH_2);
    }

    if (operatorPad.getDpad() == 180){
      tower.setTowerPosition(Position.GROUND);
    }

    if (operatorPad.getDpad() == 90) {
      tower.setTowerPosition(Position.HATCH_1);
    }

    if (operatorPad.getDpad() == 315) {
      tower.setTowerPosition(Position.CARGO_2);
    }

    if (operatorPad.getDpad() == 0) {
      tower.setTowerPosition(Position.HATCH_3);
    }

    if (operatorPad.getDpad() == 45) {
      tower.setTowerPosition(Position.CARGO_3);
    }

    //TODO hatch, rollers
    if ((!operatorPad.getButton(LogitechButton.LEFT_TRIGGER))&& (!operatorPad.getButton(LogitechButton.RIGHT_TRIGGER))) {
      tower.setRollers(Direction.NONE);
    }
    if ((operatorPad.getButton(LogitechButton.LEFT_BUMPER)) || tower.isClawOpen()){
      tower.setRollers(Direction.SLOW_IN);
    }
    if (operatorPad.getButton(LogitechButton.LEFT_TRIGGER)) { //Dpad UP
      tower.setRollers(Direction.OUT);
    }
    if (operatorPad.getButton(LogitechButton.RIGHT_TRIGGER) ) { //Dpad DOWN
      tower.setRollers(Direction.IN);
    }
  }

  private void toggleCompressor() {
    this.compressor.setClosedLoopControl(!this.compressor.enabled());
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
