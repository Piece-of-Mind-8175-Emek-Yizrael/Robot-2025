package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.CoralArm.CoralArm;
import frc.robot.subsystems.CoralArm.CoralArmIO;

public class CoralArmCommands {

    public static Command goToPosition(CoralArm coralArm, double goal) {
        CoralArmIO io = coralArm.getIO();
        return new FunctionalCommand(() -> io.resetPID(goal), () -> io.setGoal(goal),
                (interrupted) -> io.resistGravity(), io.atGoal(), coralArm).withName("Move arm to " + goal);
    }

    public static Command setVoltage(CoralArm coralArm, double voltage) {
        CoralArmIO io = coralArm.getIO();
        return coralArm.runEnd(() -> io.setVoltageWithResistGravity(voltage), () -> io.stopMotor())
                .withName("set voltage: " + voltage);
    }

}
