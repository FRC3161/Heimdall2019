package frc.robot;

public class RobotMap {

    // Drivetrain Talons (CAN)
    public static final int DRIVETRAIN_LEFT_FRONT_TALON = 1;
    public static final int DRIVETRAIN_LEFT_BACK_TALON = 2;
    public static final int DRIVETRAIN_RIGHT_FRONT_TALON = 3;
    public static final int DRIVETRAIN_RIGHT_BACK_TALON = 4;
    // PWM
    public static final int DRIVETRAIN_LEFT_COLSON = 0;
    public static final int DRIVETRAIN_RIGHT_COLSON = 1;

    // Tower controllers
    // PWM
    public static final int TOWER_ROLLER_INTAKE = 8;
    public static final int TOWER_ROLLER_WRIST = 9;
    // CAN
    public static final int ARM_CONTROLLER = 5;
    public static final int ELEVATOR_MASTER_CONTROLLER = 7;
    public static final int ELEVATOR_SLAVE_CONTROLLER = 6;
    // Relay
    public static final int LED_SPIKE = 0;
    public static final int UNDERGLOW_SPIKE = 1;
    // Solenoids (PCM)
    public static final int COLSON_SOLENOID = 0;
    public static final int CLAW_OPEN_SOLENOID = 1;
    public static final int CLAW_CLOSE_SOLENOID = 2;
    public static final int BEAK_OPEN_SOLENOID = 3;
    public static final int BEAK_CLOSE_SOLENOID = 4;

    // Sensors (DIO)
    public static final int OBJECT_SENSOR = 0;
    public static final int TOP_LIMIT_SWITCH = 2;
    public static final int BOTTOM_LIMIT_SWITCH = 3;

}