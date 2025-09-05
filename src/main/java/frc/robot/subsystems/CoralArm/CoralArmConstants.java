package frc.robot.subsystems.CoralArm;

import edu.wpi.first.math.util.Units;

public class CoralArmConstants {
    public static final int CORAL_ARM_ID = 17;
    public static final int FOLD_SWITCH = 3;

    public static final double KP = 0;
    public static final double KI = 0;
    public static final double KD = 0;
    public static final double KS = 0;
    public static final double KG = 0;
    public static final double KV = 0;
    public static final double RESIST_GRAVITY = 0;
    public static final double TOLERANCE = Units.degreesToRadians(2);

    public static final double MAX_ACCELERATION = 0;
    public static final double MAX_VELOCITY = 0;

    public static final double FORWARD_SOFT_LIMIT = 0;
    public static final double L2_POSITION = 0;
    public static final double L1_POSITION = 0;
    public static final double L3_POSITION = 0;
    public static final double L4_POSITION = 0;

    public static final double POSITION_CONVERSION_FACTOR = 1 / 21.0 /* versa */ * (2 * Math.PI) /* to radians */; // TODO
                                                                                                                   // verify

    public static double KG_OF_CORAL = 0;

    public static final boolean INVERTED = true;

    public static final int CURRENT_LIMIT = 40;

    public static final double VOLTAGE_COMPENSATION = 12.0;

}