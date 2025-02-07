package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import static frc.robot.subsystems.Transfer.TransferConstants.CORAL_INTAKE_VOLTAGE;
import static frc.robot.subsystems.Transfer.TransferConstants.CORAL_OUTTAKE_VOLTAGE;
import static frc.robot.subsystems.Transfer.TransferConstants.CORAL_INTAKE_TIME;
import frc.robot.subsystems.Transfer.Transfer;


public class TransferCommands{
          
    public static Command startTransfer(Transfer transfer) {
        return Commands.runOnce(() -> transfer.setVoltage(CORAL_INTAKE_VOLTAGE), transfer).
        andThen(new WaitUntilCommand(transfer::isCoralIn)).
        andThen(new WaitCommand(CORAL_INTAKE_TIME)).
        andThen(Commands.runOnce(transfer::stopMotor, transfer)).unless(transfer::isCoralIn);

    }
    
    
    public static Command coralOutake(Transfer transfer) {
        return Commands.startEnd(() -> transfer.setVoltage(CORAL_OUTTAKE_VOLTAGE), () -> transfer.stopMotor(),transfer).until(() -> !transfer.isCoralIn());
    }
}

