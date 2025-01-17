package frc.robot.subsystems.Transfer;

import org.littletonrobotics.junction.AutoLog;

public interface TransferIO {
  @AutoLog
  public static class TransferIOInputs {
  public double speed;
  public double voltage;
  public boolean transferSensorInput;
    
  }  

  public default void setSpeed(double speed) {}

  public default void setVoltage(double voltage) {}

  public default void stopMotor(){}

  public default boolean isCoralIn(){ return false; }

  public default void updateInputs(TransferIOInputs inputs) {}
}
