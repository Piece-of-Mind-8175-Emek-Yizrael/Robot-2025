package frc.robot.commands;

import static frc.robot.subsystems.Elevator.ElevatorConstants.CLOSE_ELEVATOR_SPEED;
import static frc.robot.subsystems.Elevator.ElevatorConstants.L2_POSITION;
import static frc.robot.subsystems.Elevator.ElevatorConstants.L3_POSITION;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.Elevator.Elevator;

public class ElevatorCommands {

    public static Command goToPosition(Elevator elevator, double position) {
        return new FunctionalCommand(() -> {
            System.out.println("going to position: " + position);
            elevator.getIO().stopMotor();
            elevator.getIO().resetPID(position);
        },
                () -> elevator.getIO().setGoal(position),
                interrupted -> {
                    elevator.getIO().stopMotor();
                    if (interrupted) {
                        System.out.println("interupted go to: " + position);
                    }
                },
                elevator.getIO().atGoal(), elevator);
    }

    public static Command stopElevator(Elevator elevator) {
        return Commands.runOnce(() -> elevator.getIO().stopMotor(), elevator);
    }

    public static Command onlyFeedForward(Elevator elevator, double velocity) {
        return Commands.run(() -> elevator.getIO().setFeedForward(velocity), elevator);
    }

    public static Command closeUntilSwitch(Elevator elevator) {
        return Commands.run(() -> elevator.getIO().setVoltageWithResistGravity(CLOSE_ELEVATOR_SPEED), elevator)
                .until(elevator.getIO()::isPressed);
    }

    public static Command goToPositionWithoutPid(Elevator elevator, double position) {
        return Commands
                .run(() -> elevator.getIO()
                        .setVoltageWithResistGravity(Math.copySign(4, position - elevator.getIO().getPosition())),
                        elevator)
                .until(() -> (Math.abs(elevator.getIO().getPosition() - position) < 1));
    }

    public static Command setSpeed(Elevator elevator, double speed) {
        return Commands.run(() -> elevator.getIO().setSpeed(speed), elevator);
    }

    public static Command closeElevator(Elevator elevator) {
        return ElevatorCommands.goToPosition(elevator, 0)
                .andThen(ElevatorCommands.closeUntilSwitch(elevator));
    }

    public static Command L2(Elevator elevator) {
        return goToPosition(elevator, L2_POSITION);
    }

    public static Command L3(Elevator elevator) {
        return goToPosition(elevator, L3_POSITION);
    }

    public static Command closeElevatorManual(Elevator elevator, double voltage) {
        return Commands.run(() -> elevator.getIO().setVoltage(voltage), elevator);
    }

    public static Command openElevatorManual(Elevator elevator, double voltage) {
        return Commands.run(() -> elevator.getIO().setVoltage(voltage), elevator);
    }

}
