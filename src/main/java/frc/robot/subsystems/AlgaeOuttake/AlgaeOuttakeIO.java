package frc.robot.subsystems.AlgaeOuttake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj2.command.Command;

public interface AlgaeOuttakeIO {
    @AutoLog
    public static class AlgaeOuttakeIOInputs {
    public double degrees;
        
    }

    public default double getDegrees(){return 0;}

    public default void setDegrees(double degrees) {}

    public default void updateInputs(AlgaeOuttakeIOInputs inputs) {}

    public default Command openArm(){
        return null;
    }

    public default Command closeArm(){
        return null;
    }

   
    
}
