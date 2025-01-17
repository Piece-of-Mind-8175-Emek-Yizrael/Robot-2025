package frc.robot.subsystems.Arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    public static class ArmIOInputs {
    public int Degrees;
        
    }
    public default void setDegrees(int Degrees) {}

    public default void updateInputs(ArmIOInputs inputs) {}
    
}
