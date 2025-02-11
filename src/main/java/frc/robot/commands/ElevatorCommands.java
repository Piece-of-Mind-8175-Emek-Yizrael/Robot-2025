package frc.robot.commands;

import static frc.robot.subsystems.Elevator.ElevatorConstants.CLOSE_ELEVATOR_SPEED;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;

public class ElevatorCommands {

    public static Command goToPosition(ElevatorSubsystem elevator, double position) {
        return Commands.startRun(() -> elevator.getIO().resetPID(),() -> elevator.getIO().setGoal(position), elevator).until(elevator.getIO().atGoal());
    }

    public static Command stopElevator(ElevatorSubsystem elevator) {
        return Commands.runOnce(() -> elevator.getIO().stopMotor(), elevator);
    }

    public static Command onlyFeedForward(ElevatorSubsystem elevator, double velocity) {
        return Commands.run(() -> elevator.getIO().setFeedForward(velocity), elevator);
    }

    public static Command closeUntilSwitch(ElevatorSubsystem elevator) {
        return Commands.run(() -> elevator.getIO().setVoltageWithResistGravity(CLOSE_ELEVATOR_SPEED), elevator)
                .until(elevator.getIO()::isPressed);
    }

    public static Command goToPositionWithoutPid(ElevatorSubsystem elevator, double position) {
        return Commands.run(() -> elevator.getIO().setVoltageWithResistGravity(Math.copySign(4, position - elevator.getIO().getPosition())), elevator)
                .until(() -> (Math.abs(elevator.getIO().getPosition() - position) < 1));
    }

    public static Command setSpeed(ElevatorSubsystem elevator, double speed) {
        return Commands.run(() -> elevator.getIO().setSpeed(speed), elevator);
    }

    public static Command closeElevator(ElevatorSubsystem elevator) {
        return ElevatorCommands.goToPosition(elevator, 0)
            .andThen(ElevatorCommands.closeUntilSwitch(elevator));
    }

}
