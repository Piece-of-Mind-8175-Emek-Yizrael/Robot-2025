package frc.robot.subsystems.Transfer;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;

import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.POM_lib.Motors.POMSparkMax;
import frc.robot.POM_lib.sensors.POMDigitalInput;

import static frc.robot.subsystems.Transfer.TransferConstants.TOLERANCE;
import static frc.robot.subsystems.Transfer.TransferConstants.TRANSFER_MOTOR_ID;
import static frc.robot.subsystems.Transfer.TransferConstants.TRANSFER_SENSOR_CHANNEL;
import static frc.robot.subsystems.drive.DriveConstants.pigeonCanId;

import java.util.function.BooleanSupplier;;



public class TransferIOReal implements TransferIO{
        private final POMDigitalInput transferSensor = new POMDigitalInput(TRANSFER_SENSOR_CHANNEL);
        private final POMSparkMax motor;
        private RelativeEncoder encoder;
        private final SparkMaxConfig config = new SparkMaxConfig();
        private ProfiledPIDController pidController;
        private TransferTunningPid pidConstants;
    
        public TransferIOReal(){
            motor = new POMSparkMax(TRANSFER_MOTOR_ID,MotorType.kBrushless);
            encoder = motor.getEncoder();
            config
            .idleMode(IdleMode.kCoast);
            motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

            pidConstants = new TransferTunningPid();

            pidController = new ProfiledPIDController(pidConstants.getKp(), pidConstants.getKi(), pidConstants.getKd(), 
                new TrapezoidProfile.Constraints(pidConstants.getMaxVelocity(), pidConstants.getMaxAcceleration()));

            pidController.setTolerance(TOLERANCE);

            encoder.setPosition(0);
        }
        
        @Override
        public void updateInputs(TransferIOInputs inputs){
            inputs.velocity = motor.get();
            inputs.voltage = (motor.getAppliedOutput() * motor.getBusVoltage());
            inputs.transferSensorInput = transferSensor.get();
            setPidValues();
            resetEncoder();
        }

        @Override
        public void setGoal(double goal) {
        pidController.setGoal(goal);
            setVoltage(pidController.calculate(encoder.getPosition()));
        }

        
        @Override
        public BooleanSupplier atGoal() {
            return () -> pidController.atGoal();
        }

        private void resetEncoder(){
            encoder.setPosition(0);
        }

        @Override
        public void setSpeed(double speed) {
        motor.set(speed);
        }
    
        @Override
        public void setVoltage(double voltage) {
            motor.setVoltage(voltage);        
        }
    
        public void stopMotor() {
            motor.stopMotor();
        }

    
        @Override
        public boolean isCoralIn() {
            return transferSensor.get();
        }

        public double getPosition(){
            return encoder.getPosition();
        }

        public void resetPID(){
            pidController.reset(encoder.getPosition(), encoder.getVelocity());
        }

        @Override
        public void setPidValues(){
            pidController.setP(pidConstants.getKp());
            pidController.setI(pidConstants.getKi());
            pidController.setD(pidConstants.getKd());
            pidController.setConstraints(new TrapezoidProfile.Constraints(pidConstants.getMaxVelocity(), pidConstants.getMaxAcceleration()));
        }

        @Override
        public void setVoltageWithPid(double voltage){
            motor.setVoltage(pidController.calculate(voltage));
        }

}
