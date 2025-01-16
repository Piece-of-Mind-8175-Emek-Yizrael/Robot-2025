package frc.robot.subsystems.Elevator;

import static frc.robot.subsystems.Elevator.ElevatorConstants.*;
import static frc.robot.util.SparkUtil.sparkStickyFault;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.ClosedLoopConfig;

import edu.wpi.first.math.filter.Debouncer;
import frc.robot.POM_lib.Motors.POMSparkMax;

public class ElevatorReal implements ElevatorIO{
    POMSparkMax motor;
    RelativeEncoder encoder = motor.getEncoder();
    private final Debouncer turnConnectedDebouncer;
    public SparkClosedLoopController controller;

    public ElevatorReal(){
        motor = new POMSparkMax(ELEVATOR_ID);
        turnConnectedDebouncer = new Debouncer(0);
        controller = motor.getClosedLoopController();
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs){
        inputs.motorConnected = true /*turnConnectedDebouncer.calculate(sparkStickyFault)*/;
        inputs.elevatorVelocity = encoder.getVelocity();
        inputs.elevatorPosition = encoder.getPosition();
        inputs.elevatorAppliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
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
    public void setSetPoint(double setpoint) {
        controller.setReference(setpoint, ControlType.kPosition);
    }
    
}
