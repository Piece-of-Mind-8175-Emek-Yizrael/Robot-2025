package frc.robot.subsystems.AlgaeTransferArm;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeTransferArmIO {
    @AutoLog
    public static class AlgaeTransferArmIOInputs {
        public double speed;
        public double voltage;
        public double position;
    }    

    public default void setSpeed(double speed){}

    public default void setVoltage(double voltage){}

    public default double getPosition(){return 0;}
    
    public default void updateInputs(AlgaeTransferArmIOInputs inputs){}

}
