package frc.robot.subsystems.Transfer;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.POM_lib.Motors.POMSparkMax;
import frc.robot.POM_lib.sensors.POMDigitalInput;
import static frc.robot.subsystems.Transfer.TransferConstants.TRANSFER_MOTOR_ID;
import static frc.robot.subsystems.Transfer.TransferConstants.TRANSFER_SENSOR_CHANNEL;;



public class TransferIOReal implements TransferIO{
        private final POMDigitalInput transferSensor = new POMDigitalInput(TRANSFER_SENSOR_CHANNEL);
        private final POMSparkMax transferMotor = new POMSparkMax(TRANSFER_MOTOR_ID,MotorType.kBrushless);
        private final SparkMaxConfig config = new SparkMaxConfig();
    
        public TransferIOReal(){
            config
            .idleMode(IdleMode.kCoast);
            transferMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        }

        @Override
        public void setSpeed(double speed) {
        transferMotor.set(speed);
        }
    
        @Override
        public void setVoltage(double voltage) {
            transferMotor.setVoltage(voltage);
        }
    
        public void stopMotor() {
            transferMotor.stopMotor();
        }

    
        @Override
        public boolean isCoralIn() {
            return transferSensor.get();
    }
    
        @Override
        public void updateInputs(TransferIOInputs inputs){
            inputs.speed = transferMotor.get();
            inputs.voltage = (transferMotor.getAppliedOutput() * transferMotor.getBusVoltage());
            inputs.transferSensorInput = transferSensor.get();
        }
    
    
}
