package frc.robot.subsystems.Elevator;

import static frc.robot.subsystems.Elevator.ElevatorConstants.*;
import java.util.function.BooleanSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.POM_lib.Motors.POMSparkMax;
import frc.robot.POM_lib.sensors.POMDigitalInput;

public class ElevatorRealPid implements ElevatorIO{
    POMSparkMax motor;
    RelativeEncoder encoder = motor.getEncoder();
    private SparkClosedLoopController controller;
    private ElevatorFeedforward feedforward;
    private double currentSetPoint;
    private POMDigitalInput foldSwitch;

    

    public ElevatorRealPid(){
        motor = new POMSparkMax(ELEVATOR_ID);
        feedforward = new ElevatorFeedforward(KS, KG, 0);
        controller = motor.getClosedLoopController();
        foldSwitch = new POMDigitalInput(FOLD_SWITCH);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs){
        inputs.motorConnected = true /*turnConnectedDebouncer.calculate(sparkStickyFault)*/;
        inputs.elevatorVelocity = encoder.getVelocity();
        inputs.elevatorPosition = encoder.getPosition();
        inputs.elevatorAppliedVolts = motor.getAppliedOutput() * motor.getBusVoltage(); //FIXME Wont Return Motor Voltage
        inputs.foldSwitch = foldSwitch.get();
        
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
        double ffPower = feedforward.calculate(encoder.getPosition() - goal > 0 ? -1 : 1);
        controller.setReference(goal, ControlType.kPosition, ClosedLoopSlot.kSlot0,ffPower, ArbFFUnits.kVoltage);
        currentSetPoint = goal;
    }

    @Override
    public BooleanSupplier atGoal() {
        return () -> Math.abs(currentSetPoint - encoder.getPosition()) < TOLERANCE;
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
    
}
