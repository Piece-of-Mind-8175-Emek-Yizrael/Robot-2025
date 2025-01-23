package frc.robot.subsystems.Elevator;

import static frc.robot.subsystems.Elevator.ElevatorConstants.*;
import java.util.function.BooleanSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.POM_lib.Motors.POMSparkMax;

public class ElevatorReal implements ElevatorIO{
    POMSparkMax motor;
    RelativeEncoder encoder = motor.getEncoder();
    private final Debouncer turnConnectedDebouncer;
    public SparkClosedLoopController controller;
    private ProfiledPIDController pidController;
    

    public ElevatorReal(){
        motor = new POMSparkMax(ELEVATOR_ID);
        turnConnectedDebouncer = new Debouncer(0);
        controller = motor.getClosedLoopController();
        pidController = new ProfiledPIDController(KP, KI, KD, null);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs){
        inputs.motorConnected = true /*turnConnectedDebouncer.calculate(sparkStickyFault)*/;
        inputs.elevatorVelocity = encoder.getVelocity();
        inputs.elevatorPosition = encoder.getPosition();
        inputs.elevatorAppliedVolts = motor.getAppliedOutput() * motor.getBusVoltage(); //FIXME Wont Return Motor Voltage
        
    }

    @Override
    public void setSpeed(double speed) {
        motor.set(speed + RESIST_GRAVITY);
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void setSetPoint(double setpoint) {
        pidController.setGoal(setpoint);
        motor.set(pidController.calculate(encoder.getPosition()));
    }

    @Override
    public void goToGoal(){
        double pidVoltage = pidController.calculate(encoder.getPosition());
        setSpeed(pidVoltage);
    }

    @Override
    public BooleanSupplier atGoal() {
        return () -> pidController.atGoal();
    }

    @Override
    public void stopMotor(){
        setVoltage(0 + RESIST_GRAVITY);
    }
    
    @Override
    public void resistGravity() {
        setVoltage(RESIST_GRAVITY);
    }
    
}
