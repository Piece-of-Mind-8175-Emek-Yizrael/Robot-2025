package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import static frc.robot.subsystems.Transfer.TransferConstants.*;
import frc.robot.subsystems.Transfer.Transfer;

public class TransferCommands {


    // public static Command startTransfer(Transfer transfer) {
    //     return Commands.runOnce(() -> transfer.setVoltage(3), transfer)
    //             .andThen(new WaitUntilCommand(transfer::isCoralIn))./*andThen(new WaitCommand(CORAL_INTAKE_TIME))*/
    //             until(() -> !transfer.isCoralIn())
    //             .andThen(Commands.runOnce(transfer::stopMotor, transfer));

    // }


    
    // public static Command coralOutake(Transfer transfer) {
    //     return Commands.startEnd(() -> transfer.setVoltage(CORAL_OUTTAKE_VOLTAGE), () -> transfer.stopMotor(), transfer)
    //             .until(() -> !transfer.isCoralIn());
    // }

    public static Command coralOutake(Transfer transfer) {
        return Commands.startEnd(() -> transfer.setVoltage(CORAL_OUTTAKE_VOLTAGE), () -> transfer.stopMotor(), transfer);
    }

    public static Command intakeCoral(Transfer transfer){
        return Commands.startEnd(() -> transfer.setVoltage(3), () -> transfer.stopMotor() ,transfer);
    }

    public static Command takeCoralIn(Transfer transfer){
        return Commands.startEnd(() -> transfer.setVoltage(-3) , transfer :: stopMotor, transfer);
    }
}
