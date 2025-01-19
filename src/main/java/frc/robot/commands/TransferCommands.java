package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import static frc.robot.subsystems.Transfer.TransferConstants.CORAL_INTAKE_SPEED;
import static frc.robot.subsystems.Transfer.TransferConstants.CORAL_OUTTAKE_SPEED;
import frc.robot.subsystems.Transfer.Transfer;


public class TransferCommands{
          
    public static Command startTransfer(Transfer transfer) {
        return Commands.startEnd(() -> transfer.setSpeed(CORAL_INTAKE_SPEED), () ->transfer.stopMotor()).until(() -> transfer.isCoralIn());
    }
    
    
    public static Command coralOutake(Transfer transfer) {
        return Commands.startEnd(() -> transfer.setSpeed(CORAL_OUTTAKE_SPEED), () -> transfer.stopMotor()).until(() -> !transfer.isCoralIn());
    }
}

