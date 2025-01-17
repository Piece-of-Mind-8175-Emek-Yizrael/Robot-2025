package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Transfer.Transfer;
import static frc.robot.subsystems.Transfer.TransferConstants.CORAL_INTAKE_SPEED;
import static frc.robot.subsystems.Transfer.TransferConstants.CORAL_OUTTAKE_SPEED;


public class TransferCommands extends SubsystemBase {
Transfer transfer;

    public Command startTransfer() {
        return startEnd(() -> transfer.setSpeed(CORAL_INTAKE_SPEED), () ->transfer.stopMotor()).until(() -> transfer.getIO().isCoralIn());
            }
    

            public Command coralOutake() {
                return startEnd(() -> transfer.setSpeed(CORAL_OUTTAKE_SPEED), () -> transfer.stopMotor()).until(() -> !transfer.getIO().isCoralIn());
            }
            
}
