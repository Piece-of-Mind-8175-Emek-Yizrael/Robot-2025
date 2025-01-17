package frc.robot.subsystems.Transfer;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.POM_lib.Motors.POMSparkMax;

public class TransferIOReal implements TransferIO{
        private final DigitalInput IRSensor1 = new DigitalInput(0);
        //private final DigitalInput IRSensor2 = new DigitalInput(0);
        private final POMSparkMax transferMotor = new POMSparkMax(0,MotorType.kBrushless);
        private final SparkMaxConfig config = new SparkMaxConfig();
        boolean IRS1isCrossed;
    
        public TransferIOReal(){
            config
            .idleMode(IdleMode.kCoast);
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
            return IRSensor1.get();
    }
    
        @Override
        public void updateInputs(TransferIOInputs inputs){
            inputs.velocity = transferMotor.get();
            inputs.voltage = (transferMotor.getAppliedOutput() * transferMotor.getBusVoltage());
            inputs.IRSensor1isCrossed = IRSensor1.get();
            inputs.current = transferMotor.getOutputCurrent();
        }
    
    
}
