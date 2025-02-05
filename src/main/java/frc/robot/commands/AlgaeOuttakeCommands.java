package frc.robot.commands;
import static frc.robot.subsystems.AlgaeOuttake.AlgaeOuttakeConstants.ARM_OPEN_DEGREE;
import static frc.robot.subsystems.AlgaeOuttake.AlgaeOuttakeConstants.ARM_CLOSED_DEGREE;
import static frc.robot.subsystems.AlgaeOuttake.AlgaeOuttakeConstants.ARM_OPEN_TIME;
import static frc.robot.subsystems.AlgaeOuttake.AlgaeOuttakeConstants.ARM_CLOSE_TIME;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.AlgaeOuttake.AlgaeOuttake;


public class AlgaeOuttakeCommands {

    public static Command openArm(AlgaeOuttake algaeOuttake){
        return Commands.runOnce(()-> algaeOuttake.setDegrees(100),algaeOuttake).andThen(new WaitCommand(ARM_OPEN_TIME));
    }

    public static Command closeArm(AlgaeOuttake algaeOuttake){
        return Commands.runOnce(()-> algaeOuttake.setDegrees(180),algaeOuttake).andThen(new WaitCommand(ARM_CLOSE_TIME));
    }
    
}
