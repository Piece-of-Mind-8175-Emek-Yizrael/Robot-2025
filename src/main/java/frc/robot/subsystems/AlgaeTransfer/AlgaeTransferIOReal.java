package frc.robot.subsystems.AlgaeTransfer;

import frc.robot.POM_lib.Motors.POMTalonFX;
import static frc.robot.subsystems.AlgaeTransfer.AlgaeTransferConstants.*;


public class AlgaeTransferIOReal implements AlgaeTransferIO {
    private POMTalonFX motor;

    public AlgaeTransferIOReal(){
        motor = new POMTalonFX(MOTOR_ID);
    }

    @Override
    public void setSpeed(double speed) {
        motor.set(speed);
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void stopMotor() {
        motor.stopMotor();
    }
}
