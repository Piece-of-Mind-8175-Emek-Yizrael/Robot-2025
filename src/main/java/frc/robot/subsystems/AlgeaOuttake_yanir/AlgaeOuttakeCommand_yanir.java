package frc.robot.subsystems.AlgeaOuttake_yanir;

import edu.wpi.first.wpilibj2.command.Command;

public class AlgaeOuttakeCommand_yanir extends Command {
    private final AlgaeOuttakeSubsystem algaeOuttakeSubsystem;
    double ServoDegrees;

    public AlgaeOuttakeCommand_yanir(AlgaeOuttakeSubsystem algaeOuttakeSubsystem, double servoDegrees) {
        this.algaeOuttakeSubsystem = algaeOuttakeSubsystem;
        addRequirements(algaeOuttakeSubsystem);

        ServoDegrees = servoDegrees;

    }

    @Override
    public void initialize() {
        algaeOuttakeSubsystem.setDegrees(ServoDegrees);

    }

    int executeCounter = 0;

    @Override
    public void execute() {

        algaeOuttakeSubsystem.setDegrees(0);

        // wait TODO: add a delay here to allow the servo to move before setting it to 0
        // again
        // This is a placeholder for the actual delay logic

        algaeOuttakeSubsystem.setDegrees(ServoDegrees);

        executeCounter = +1;

    }

    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        if (executeCounter >= 10) {
            return true;
        } else {
            return false;

        }
    }
}