package frc.robot;

public class RobotMap {

    // Drivetrain Talons (CAN)
    public static final int DRIVETRAIN_LEFT_FRONT_TALON = 1;
    public static final int DRIVETRAIN_LEFT_BACK_TALON = 3;
    public static final int DRIVETRAIN_RIGHT_FRONT_TALON = 2;
    public static final int DRIVETRAIN_RIGHT_BACK_TALON = 4;
    // PWM
    public static final int DRIVETRAIN_LEFT_COLSON = 0;
    public static final int DRIVETRAIN_RIGHT_COLSON = 1;

    // Tower controllers
    // PWM
    public static final int TOWER_ROLLER_1 = 9;
    public static final int TOWER_ROLLER_2 = 10;
    public static final int TOP_LIMIT_SWITCH = 7;
    public static final int BOTTOM_LIMIT_SWITCH = 8;
    // CAN
    public static final int ARM_CONTROLLER = 5;
    public static final int ELEVATOR_MASTER_CONTROLLER = 6;
    public static final int ELEVATOR_SLAVE_CONTROLLER = 7;
    // Relay
    public static final int LED_SPIKE = 0;
    public static final int UNDERGLOW_SPIKE = 1;
    // Solenoids (PCM)
    public static final int COLSON_SOLENOID = 0;
    public static final int CLAW_SOLENOID = 1;
    public static final int BEAK_SOLENOID = 2;

    // Sensors (DIO)
    public static final int BALL_SENSOR = 0;
    public static final int HATCH_SENSOR = 1;

}