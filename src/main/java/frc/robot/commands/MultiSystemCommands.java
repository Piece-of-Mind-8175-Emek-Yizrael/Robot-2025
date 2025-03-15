package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.AlgaeOuttake.AlgaeOuttake;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.FieldConstants;

public class MultiSystemCommands {

    public static Command ClearAlgeaHigh(Drive drive, Elevator elevator, AlgaeOuttake algaeOuttake) {
        return Commands.sequence(
                AutonomousRoutines.driveToPoseInCorrectAlliance(drive, FieldConstants.Reef.redAlgaePositions[0], false),
                ElevatorCommands.goToPosition(elevator, 15),
                Commands.parallel(ElevatorCommands.goToPosition(elevator, 25),
                        DriveCommands.joystickDriveRobotRelative(drive, () -> -0.5, () -> 0,
                                () -> 0.3)
                                .withTimeout(1)),
                AlgaeOuttakeCommands.closeArm(algaeOuttake)

        );
    }

}
