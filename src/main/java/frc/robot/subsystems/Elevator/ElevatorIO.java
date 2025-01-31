package frc.robot.subsystems.Elevator;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
 
    @AutoLog
    public static class ElevatorIOInputs  {
        
        boolean motorConnected = false;
        double elevatorVelocity = 0.0;
        double elevatorPosition = 0.0;
        double elevatorAppliedVolts = 0.0;
        boolean foldSwitch = false; 
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}
    public default void setSpeed(double speed) {}
    public default void setVoltage(double voltage) {}
    public default void setGoal(double goal) {}
    public default BooleanSupplier atGoal(){ return () ->  false; }
    public default void stopMotor(){}
    public default void resistGravity(){}
    public default void resetlfPressed(){}
}
