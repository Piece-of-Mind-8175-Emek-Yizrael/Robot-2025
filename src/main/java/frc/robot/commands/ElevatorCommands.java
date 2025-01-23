package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorCommands {


    // public static Command goToPosition(double position){
    //     return run(() -> setSetPoint(position)).until(atGoal());
    // }
    
    // public static Command stopElevator(){
    //     return runOnce(() -> resistGravity());
    // }

    public static Command goToPosition(ElevatorSubsystem elevator, double position){
        return Command.run(() -> elevator.setSetPoint(position)).until(() -> elevator.atGoal());
    }
    
}
