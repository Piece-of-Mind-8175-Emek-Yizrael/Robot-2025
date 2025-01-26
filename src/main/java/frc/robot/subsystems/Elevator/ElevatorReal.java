package frc.robot.subsystems.Elevator;

import static frc.robot.subsystems.Elevator.ElevatorConstants.ELEVATOR_ID;
import static frc.robot.subsystems.Elevator.ElevatorConstants.FOLD_SWITCH;
import static frc.robot.subsystems.Elevator.ElevatorConstants.KD;
import static frc.robot.subsystems.Elevator.ElevatorConstants.KG;
import static frc.robot.subsystems.Elevator.ElevatorConstants.KI;
import static frc.robot.subsystems.Elevator.ElevatorConstants.KP;
import static frc.robot.subsystems.Elevator.ElevatorConstants.KS;
import static frc.robot.subsystems.Elevator.ElevatorConstants.KV;
import static frc.robot.subsystems.Elevator.ElevatorConstants.MAX_ACCELERATION;
import static frc.robot.subsystems.Elevator.ElevatorConstants.MAX_VELOCITY;
import static frc.robot.subsystems.Elevator.ElevatorConstants.RESIST_GRAVITY;
import static frc.robot.subsystems.Elevator.ElevatorConstants.TOLERANCE;

import java.util.function.BooleanSupplier;

import com.revrobotics.RelativeEncoder;

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

    

    public ElevatorReal(){
        motor = new POMSparkMax(ELEVATOR_ID);
        feedforward = new ElevatorFeedforward( KS, KG, KV);
        pidController = new ProfiledPIDController(KP, KI, KD, new TrapezoidProfile.Constraints(MAX_VELOCITY,MAX_ACCELERATION));
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
        pidController.setGoal(goal);
        setVoltage(pidController.calculate(encoder.getPosition()) + feedforward.calculate(pidController.getSetpoint().velocity));
    }

    @Override
    public BooleanSupplier atGoal() {
        pidController.setTolerance(TOLERANCE);//TODO chaeck this
        return () -> pidController.atGoal();
    }

    @Override
    public void stopMotor(){
        setVoltage(0 + feedforward.calculate(0));//TODO check this 
    }
    
    @Override
    public void resistGravity() {
        setVoltage(RESIST_GRAVITY);
    }

    @Override
    public void resetlfPressed() {
        if(foldSwitch.get()){
            encoder.setPosition(0);
        }
    }
    
}
