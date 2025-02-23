package frc.robot.subsystems.Transfer;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLog;

public interface TransferIO {
  @AutoLog
  public static class TransferIOInputs {
  public double velocity;
  public double voltage;
  public double current;
  public boolean transferSensorInput;
    
  }  

  public default void setSpeed(double speed) {}

  public default void setVoltage(double voltage) {}

  public default void stopMotor() {}

  public default boolean isCoralIn() { return false; }

  public default void updateInputs(TransferIOInputs inputs) {}

  public default void setPidValues() {}

  public default void setGoal(double goal) {}

  public default BooleanSupplier atGoal() { return () -> false; }

  public default void setVoltageWithPid(double voltage) {}
}
