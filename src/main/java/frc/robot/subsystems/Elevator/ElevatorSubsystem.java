package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.POM_lib.Dashboard.DashboardNumber;

import static frc.robot.subsystems.Elevator.ElevatorConstants.KP;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ElevatorSubsystem extends SubsystemBase {
    private  ElevatorIO elevatorIO;
    public  ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();


    public ElevatorSubsystem(ElevatorIO elevatorIO){
       this.elevatorIO = elevatorIO;

    }
    
    public void periodic(){
        elevatorIO.updateInputs(elevatorInputs);
        Logger.processInputs("Elevator/elevator", elevatorInputs);
    
    }

    
    public void setSpeed(double speed){
        elevatorIO.setSpeed(speed);
    }

    public void setVoltage(double voltage){
        elevatorIO.setVoltage(voltage);
    }

    public void setGoal(double goal){
        elevatorIO.setGoal(goal);
    }

    public BooleanSupplier atGoal(){
       return elevatorIO.atGoal();
    }

    public void resistGravity(){
        elevatorIO.resistGravity();
    }

    public void stopElevator(){
        elevatorIO.stopMotor();
    }

    public void resetlfPressed(){
        elevatorIO.resetlfPressed();
    }


}
