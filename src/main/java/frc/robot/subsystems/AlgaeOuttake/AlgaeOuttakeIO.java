package frc.robot.subsystems.AlgaeOuttake;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeOuttakeIO {
    @AutoLog
    public static class AlgaeOuttakeIOInputs {
    public double degrees;
        
    }

    public default double getDegrees(){return 0;}

    public default void setDegrees(double degrees) {}

    public default void updateInputs(AlgaeOuttakeIOInputs inputs) {}

    
}
