package frc.robot.commands;
import static frc.robot.subsystems.AlgaeOuttake.AlgaeOuttakeConstants.ARM_OPEN_DEGREE;
import static frc.robot.subsystems.AlgaeOuttake.AlgaeOuttakeConstants.ARM_CLOSED_DEGREE;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.AlgaeOuttake.AlgaeOuttake;


public class AlgaeOuttakeCommands {

    public static Command openArm(AlgaeOuttake algaeOuttake){
        return Commands.run(()-> algaeOuttake.setDegrees(ARM_OPEN_DEGREE),algaeOuttake).until(()-> algaeOuttake.getDegrees() == ARM_OPEN_DEGREE);
    }

    public static Command closeArm(AlgaeOuttake algaeOuttake){
        return Commands.runOnce(()-> algaeOuttake.setDegrees(ARM_CLOSED_DEGREE),algaeOuttake).until(()-> algaeOuttake.getDegrees() == ARM_CLOSED_DEGREE);
    }
    
}
