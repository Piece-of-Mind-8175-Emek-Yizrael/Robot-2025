package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorCommands {
    
    public static Command goToPosition(ElevatorSubsystem elevator, double position){
        System.out.print("Going To: " + position);
        return Commands.run(() -> elevator.setGoal(position), elevator).until(elevator.atGoal());
    }

    public static Command stopElevator(ElevatorSubsystem elevator){
        return Commands.runOnce(elevator::stopElevator, elevator);
    }
    
}
