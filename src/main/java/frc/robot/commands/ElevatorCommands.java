package frc.robot.commands;

import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorCommands {
    
    public static Command goToPosition(ElevatorSubsystem elevator, double position){
        return Commands.run(() -> elevator.getIO().setGoal(position), elevator).until(elevator.getIO().atGoal());
    }

    public static Command stopElevator(ElevatorSubsystem elevator){
        return Commands.runOnce(() -> elevator.getIO().stopMotor(), elevator);
    }

    public static Command onlyFeedForward(ElevatorSubsystem elevator, double velocity){
        return Commands.run(() -> elevator.getIO().setFeedForward(velocity), elevator).until(elevator.getIO().atGoal());
    }

    public static Command moveDown(ElevatorSubsystem elevator, double voltage){
        return Commands.run(() -> elevator.getIO().setVoltageWithResistGravity(voltage), elevator).until(elevator.getIO()::isPressed);
    }

    public static Command setSpeed(ElevatorSubsystem elevator, double speed){
        return Commands.run(() -> elevator.getIO().setSpeed(speed), elevator);
    }
    
}
