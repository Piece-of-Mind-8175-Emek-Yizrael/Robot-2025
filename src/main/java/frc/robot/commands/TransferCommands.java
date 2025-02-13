package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import static frc.robot.subsystems.Transfer.TransferConstants.CORAL_INTAKE_VOLTAGE;
import static frc.robot.subsystems.Transfer.TransferConstants.CORAL_OUTTAKE_VOLTAGE;
import static frc.robot.subsystems.Transfer.TransferConstants.CORAL_INTAKE_TIME;
import frc.robot.subsystems.Transfer.Transfer;

public class TransferCommands {

    public static Command startTransfer(Transfer transfer) {
        return Commands.waitUntil(transfer::isCoralIn)
                .andThen(() -> transfer.setVoltage(CORAL_INTAKE_VOLTAGE))
                .andThen(Commands.waitUntil(() -> !transfer.isCoralIn()))
                .andThen(Commands.runOnce(transfer::stopMotor, transfer));

    }

    public static Command coralOutake(Transfer transfer) {
        return Commands.startEnd(() -> transfer.setVoltage(CORAL_OUTTAKE_VOLTAGE), () -> transfer.stopMotor(), transfer)
                .withTimeout(CORAL_INTAKE_TIME);
    }
}
