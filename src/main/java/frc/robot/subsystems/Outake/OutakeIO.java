package frc.robot.subsystems.Outake;

import org.littletonrobotics.junction.AutoLog;

public interface OutakeIO {
    
    @AutoLog
    public static class OutakeIOInputs {
        public boolean outakeConnected = false;
        public double outakePositionRad = 0.0;
        public double outakeVelocityRadPerSec = 0.0;
        public double outakeAppliedVolts = 0.0;
        public double outakeCurrentAmps = 0.0;

       
    }

    public default void setSpeed(double speed) {}

    public default void stop() {}

    public default void updateInputs(OutakeIOInputs inputs) {}
}
