package frc.robot.subsystems.Arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    public static class ArmIOInputs {
    public double Degrees;
        
    }
    public default void setDegrees(Double Degrees) {}

    public default void updateInputs(ArmIOInputs inputs) {}
    
}
