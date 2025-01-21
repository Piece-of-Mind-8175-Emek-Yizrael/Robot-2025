package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {
    public final ElevatorIO elevatorIO;
    public final ElevatorIOInputsAutoLogged elevatorInputs;
    

    public ElevatorSubsystem(ElevatorIO elevatorIO){
        this.elevatorIO = elevatorIO;
    }
    
    public void periodic(){
        elevatorIO.updateInputs(elevatorInputs);
        Logger.processInputs("Elevator/elevator", elevatorInputs);
    }

    
    public ElevatorIO getIO(){
        return elevatorIO;
    }

    
    public void setSpeed(double speed){
        elevatorIO.setSpeed(speed);
    }

    public void setVoltage(double voltage){
        elevatorIO.setVoltage(voltage);
    }

    public void setSetPoint(double setpoint){
        elevatorIO.setSetPoint(setpoint);
    }

    public void goToGoal(){
        elevatorIO.goToGoal();
    }

    public BooleanSupplier atGoal(){
        elevatorIO.atGoal();
    }

    public Command goToPosition(double position){
        return run(() -> setSetPoint(position)).until(atGoal());
    }
}
