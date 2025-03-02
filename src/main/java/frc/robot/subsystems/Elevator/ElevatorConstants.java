package frc.robot.subsystems.Elevator;

public class ElevatorConstants {
    public static final int ELEVATOR_ID = 16;
    public static final int FOLD_SWITCH = 1;
    public static final int BRAKE_SWITCH = 2;

    public static final double KP = 1.2;
    public static final double KI = 0;
    public static final double KD = 0;
    public static final double KS = 0.19;
    public static final double KG = 0.31;
    public static final double UPPER_KG = 0.15;
    public static final double KV = 0;
    public static final double RESIST_GRAVITY = 0;
    public static final double TOLERANCE = 0.15;

    public static final double MAX_ACCELERATION = 75;
    public static final double MAX_VELOCITY = 50;

    public static final double FORWARD_SOFT_LIMIT = 0;
    public static final double L2_POSITION = 34;
    public static final double L3_POSITION = 55;

    public static final double CLOSE_ELEVATOR_SPEED = -0.5;

    public static final double POSITION_CONVERSION_FACTOR = 1;

    public static double KG_OF_CORAL = 0.1;

    public static final boolean INVERTED = true;

    public static final int CURRENT_LIMIT = 40;

    public static final double VOLTAGE_COMPENSATION = 12.0;

    public static final double UPPER_POSITION = 20;

    public static final double MANUAL_SLOW_OPEN = 2;
    public static final double MANUAL_SLOW_CLOSE = -1;
    public static final double MANUAL_FAST_OPEN = 3.5;
    public static final double MANUAL_FAST_CLOSE = -3;
}
