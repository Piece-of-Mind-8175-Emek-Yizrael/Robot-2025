package frc.robot.subsystems.AlgaeTransferArm;

import com.revrobotics.RelativeEncoder;

import frc.robot.POM_lib.Motors.POMSparkMax;
import static frc.robot.subsystems.AlgaeTransferArm.AlgaeTransferArmConstants.*;


public class AlgaeTransferArmIOReal implements AlgaeTransferArmIO {
    private POMSparkMax motor;
    private RelativeEncoder encoder;

    public AlgaeTransferArmIOReal(){
        motor = new POMSparkMax(MOTOR_ID);
        encoder = motor.getEncoder();
    }

    @Override
    public void updateInputs(AlgaeTransferArmIOInputs inputs) {
        inputs.position = encoder.getPosition();
        inputs.speed = motor.get();
        inputs.voltage = motor.getAppliedOutput() * motor.getBusVoltage();
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
    public double getPosition() {
        return encoder.getPosition();   
    }
}
