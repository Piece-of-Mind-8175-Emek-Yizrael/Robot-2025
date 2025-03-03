package frc.robot.subsystems.AlgaeTransfer;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface AlgaeTransferIO {
    @AutoLog
    public static class AlgaeTransferIOInputs{
    public double voltage;
    public double speed;        
    }

    public default void setSpeed(double speed){}

    public default void setVoltage(double voltage){}

    public default void stopMotor(){}

    public default void updateInputs(AlgaeTransferIOInputs inputs){}

}