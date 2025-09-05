package frc.robot.subsystems.CoralArm;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLog;

public interface CoralArmIO {

    @AutoLog
    public static class CoralArmIOInputs {

        boolean motorConnected = false;
        double coralArmVelocity = 0.0;
        double coralArmPosition = 0.0;
        double coralArmAppliedVolts = 0.0;
        boolean foldSwitch = false;
    }

    public default void updateInputs(CoralArmIOInputs inputs) {
    }

    public default void setPercentage(double speed) {
    }

    public default void setVoltage(double voltage) {
    }

    public default void setVoltageWithCoral(double voltage) {
    }

    public default void setGoal(double goal) {
    }

    public default BooleanSupplier atGoal() {
        return () -> false;
    }

    public default void stopMotor() {
    }

    public default void resistGravity() {
    }

    public default void resetIfPressed() {
    }

    public default void setFeedForward(double voltage) {
    }

    public default boolean isPressed() {
        return false;
    }

    public default void setVoltageWithResistGravity(double voltage) {
    }

    public default double getPosition() {
        return 0;
    }

    public default void resetPID() {
    }

    public default void resetPID(double newGoal) {
    }

}