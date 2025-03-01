package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Transfer.Transfer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.FieldConstants;

public class AutonomousRoutines {
    public static Command putL2Right(Drive drive, Elevator elevator, Transfer transfer) {
        Pose2d[] pose1 = new Pose2d[] { new Pose2d(13.5, 1.5, new Rotation2d(Math.PI)),
                new Pose2d(16.4, 1.1, Rotation2d.fromDegrees(125)) };
        return Commands.sequence(
                new DriveCommands.DriveToPosition(drive, FieldConstants.Reef.redRightBranches[2]).withTimeout(4)
                        .alongWith(ElevatorCommands.goToPosition(elevator, 8)),
                DriveCommands.joystickDriveRobotRelative(drive, () -> 0.35, () -> 0, () -> 0).withTimeout(0.3),
                ElevatorCommands.L2(elevator),
                TransferCommands.coralOutakeFast(transfer).withTimeout(0.5),
                Commands.parallel(
                        ElevatorCommands.closeElevator(elevator),
                        new DriveCommands.DriveToPosition(drive, pose1[0])
                                .until(() -> drive.getPose().getTranslation()
                                        .getDistance(pose1[0].getTranslation()) < 0.5)),
                new DriveCommands.DriveToPosition(drive, pose1[1]).withTimeout(3),
                TransferCommands.intakeCoral(transfer));
    }

}
