package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {
    private  ElevatorIO elevatorIO;
    public  ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();


    public ElevatorSubsystem(ElevatorIO elevatorIO){
       this.elevatorIO = elevatorIO;

       setDefaultCommand(this.run(elevatorIO::resistGravity));

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
