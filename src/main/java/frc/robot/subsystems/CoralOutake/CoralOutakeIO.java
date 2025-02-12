package frc.robot.subsystems.CoralOutake;

import org.littletonrobotics.junction.AutoLog;

public interface CoralOutakeIO {
    @AutoLog
    public static class CoralOutakeIOInputs {
      boolean isCoralIn = false;
    }

    public default void updateInputs(CoralOutakeIOInputs inputs){}
    
    public default void setPower(double power){};

    public default boolean isCoralIn(){
        return true;
    }
}
