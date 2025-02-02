package frc.robot.subsystems.Elevator;

import static frc.robot.subsystems.Elevator.ElevatorConstants.*;


import java.util.function.BooleanSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.POM_lib.Motors.POMSparkMax;
import frc.robot.POM_lib.sensors.POMDigitalInput;

public class ElevatorReal implements ElevatorIO{
    POMSparkMax motor;
    RelativeEncoder encoder = motor.getEncoder();
    private ProfiledPIDController pidController;
    private ElevatorFeedforward feedforward;
    private POMDigitalInput foldSwitch;
    private ElevatorTuningPid pidConstants;
    private SoftLimitConfig softLimit;


    
    public ElevatorReal(){
        motor = new POMSparkMax(ELEVATOR_ID);
        feedforward = new ElevatorFeedforward( KS, KG, KV);
        pidController = new ProfiledPIDController(KP, KI, KD, new TrapezoidProfile.Constraints(MAX_VELOCITY,MAX_ACCELERATION));
        // feedforward = new ElevatorFeedforward( pidConstants.getKs(), pidConstants.getKg(), pidConstants.getKv());
        // pidController = new ProfiledPIDController(pidConstants.getKp(), pidConstants.getKi(), pidConstants.getKd(), new TrapezoidProfile.Constraints(pidConstants.getMaxVelocity(),pidConstants.getMaxAcceleration()));

        foldSwitch = new POMDigitalInput(FOLD_SWITCH);
        pidController.setTolerance(TOLERANCE);//TODO chaeck this

        SparkMaxConfig config = new SparkMaxConfig();
        config.softLimit.forwardSoftLimit(FORWARD_SOFT_LIMIT);
        config.encoder.positionConversionFactor(POSITION_CONVERSION_FACTOR).velocityConversionFactor(POSITION_CONVERSION_FACTOR / 60.0);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs){
        inputs.motorConnected = true /*turnConnectedDebouncer.calculate(sparkStickyFault)*/;
        inputs.elevatorVelocity = encoder.getVelocity();
        inputs.elevatorPosition = encoder.getPosition();
        inputs.elevatorAppliedVolts = motor.getAppliedOutput() * motor.getBusVoltage(); //FIXME Wont Return Motor Voltage
        inputs.foldSwitch = foldSwitch.get();
        setPidValues();
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
    public void setGoal(double goal) {
        pidController.setGoal(goal);
        setVoltage(pidController.calculate(encoder.getPosition()) + feedforward.calculate(pidController.getSetpoint().velocity));
    }

    @Override
    public BooleanSupplier atGoal() {
        return () -> pidController.atGoal();
    }

    @Override
    public void stopMotor(){
        setVoltage(0 + feedforward.calculate(0));//TODO check this 
    }
    
    @Override
    public void resistGravity() {
        setVoltage(feedforward.calculate(0));//TODO check this 
    }

    @Override
    public void resetlfPressed() {
        if(foldSwitch.get()){
            encoder.setPosition(0);
        }
    }

    @Override
    public void setPidValues(){
        pidController.setP(pidConstants.getKp());
        pidController.setI(pidConstants.getKi());
        pidController.setD(pidConstants.getKd());
        pidController.setConstraints(new TrapezoidProfile.Constraints(pidConstants.getMaxVelocity(), pidConstants.getMaxAcceleration()));
        feedforward = new ElevatorFeedforward(pidConstants.getKs(), pidConstants.getKg(), pidConstants.getKv());
    }

    @Override
    public void setFeedForward(double velocity){
        motor.setVoltage(feedforward.calculate(velocity));
    }

    @Override
    public boolean isPressed() {
        return foldSwitch.get();
    }

    @Override
    public void setVoltageWithResistGravity(double voltage) {
        motor.setVoltage(feedforward.calculate(0) + voltage);
    }
    
}
