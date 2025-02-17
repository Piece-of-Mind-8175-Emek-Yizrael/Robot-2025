package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
    private  ElevatorIO elevatorIO;
    public  ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();


    public Elevator(ElevatorIO elevatorIO){
       this.elevatorIO = elevatorIO;

       setDefaultCommand(new RepeatCommand(new ConditionalCommand(this.runOnce(()->elevatorIO.setVoltage(0)), this.runOnce(elevatorIO::resistGravity), elevatorIO::isPressed)).beforeStarting(new PrintCommand("Elevator default command")));

    }
    
    public void periodic(){
        elevatorIO.updateInputs(elevatorInputs);
        Logger.processInputs("Elevator/elevator", elevatorInputs);
    
    }

    
    // public void setSpeed(double speed){
    //     elevatorIO.setSpeed(speed);
    // }

    // public void setVoltage(double voltage){
    //     elevatorIO.setVoltage(voltage);
    // }

    // public void setGoal(double goal){
    //     elevatorIO.setGoal(goal);
    // }

    // public BooleanSupplier atGoal(){
    //    return elevatorIO.atGoal();
    // }

    // public void resistGravity(){
    //     elevatorIO.resistGravity();
    // }

    // public void stopElevator(){
    //     elevatorIO.stopMotor();
    // }

    // public void resetlfPressed(){
    //     elevatorIO.resetlfPressed();
    // }

    public ElevatorIO getIO() {
        return elevatorIO;
    }

}
