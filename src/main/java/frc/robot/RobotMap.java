package frc.robot;

public class RobotMap {

    // Drivetrain Talons (CAN)
    public static final int DRIVETRAIN_LEFT_FRONT_TALON = 1;
    public static final int DRIVETRAIN_LEFT_BACK_TALON = 3;
    public static final int DRIVETRAIN_RIGHT_FRONT_TALON = 2;
    public static final int DRIVETRAIN_RIGHT_BACK_TALON = 4;
    public static final int DRIVETRAIN_LEFT_COLSON = 5;
    public static final int DRIVETRAIN_RIGHT_COLSON = 6;

    // Tower controllers
    // PWM
    public static final int TOWER_ROLLER_1 = 9;
    public static final int TOWER_ROLLER_2 = 10;
    // CAN
    public static final int ARM_CONTROLLER = 7;
    public static final int ELEVATOR_CONTROLLER = 8;
    // Relay
    public static final int LED_SPIKE = 0;

    // Solenoids (PCM)
    public static final int COLSON_SOLENOID = 0;
    public static final int CLAW_SOLENOID = 1;
    public static final int BEAK_SOLENOID = 2;

    // Sensors (DIO)
    public static final int BALL_SENSOR = 0;
    public static final int HATCH_SENSOR = 1;

}