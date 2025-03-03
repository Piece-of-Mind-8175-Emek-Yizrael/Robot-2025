package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.AlgaeTransfer.AlgaeTransfer;
import static frc.robot.subsystems.AlgaeTransfer.AlgaeTransferConstants.*;


public class AlgaeTransferCommands {
    
    public static Command intakeAlgae(AlgaeTransfer algaeTransfer){
        return Commands.runEnd( ()-> algaeTransfer.getIO().setSpeed(INTAKE_VOLTAGE),
         ()-> algaeTransfer.getIO().stopMotor(), algaeTransfer);
    }

    public static Command OuttakeAlgae(AlgaeTransfer algaeTransfer){
        return Commands.runEnd( ()-> algaeTransfer.getIO().setSpeed(OUTTAKE_VOLTAGE),
         ()-> algaeTransfer.getIO().stopMotor(), algaeTransfer);
    }

}
