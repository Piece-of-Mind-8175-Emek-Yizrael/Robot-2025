package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.AlgaeOuttake.AlgaeOuttake;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.FieldConstants;

public class MultiSystemCommands {

        public static Command ClearAlgeaLow(Drive drive, Elevator elevator, AlgaeOuttake algaeOuttake) {
                return Commands.sequence(
                                Commands.parallel(
                                                new DriveCommands.DriveToSuppliedPosition(drive,
                                                                () -> getClosestReefAlgae(drive.getPose())).andThen(
                                                                                DriveCommands.joystickDriveRobotRelative(
                                                                                                drive, () -> 0.4,
                                                                                                () -> 0, () -> 0)
                                                                                                .withTimeout(0.3)),
                                                AlgaeOuttakeCommands.openArm(algaeOuttake),
                                                ElevatorCommands.goToPosition(elevator, 2)),
                                ElevatorCommands.goToPosition(elevator, 15).withTimeout(0.5),
                                Commands.parallel(ElevatorCommands.goToPosition(elevator, 25).withTimeout(1),
                                                DriveCommands.joystickDriveRobotRelative(drive, () -> -0.5, () -> 0,
                                                                () -> 0.3)
                                                                .withTimeout(1)),
                                AlgaeOuttakeCommands.closeArm(algaeOuttake)

                );

        }

        public static Command ClearAlgeaHigh(Drive drive, Elevator elevator, AlgaeOuttake algaeOuttake) {
                return Commands.sequence(
                                Commands.parallel(
                                                new DriveCommands.DriveToSuppliedPosition(drive,
                                                                () -> getClosestReefAlgae(drive.getPose())).andThen(
                                                                                DriveCommands.joystickDriveRobotRelative(
                                                                                                drive, () -> 0.4,
                                                                                                () -> 0, () -> 0)
                                                                                                .withTimeout(0.3)),
                                                AlgaeOuttakeCommands.openArm(algaeOuttake),
                                                ElevatorCommands.goToPosition(elevator, 23).withTimeout(1)),
                                ElevatorCommands.goToPosition(elevator, 33).withTimeout(1),
                                Commands.parallel(ElevatorCommands.goToPosition(elevator, 40).withTimeout(.9),
                                                DriveCommands.joystickDriveRobotRelative(drive, () -> -0.5, () -> 0,
                                                                () -> 0.3)
                                                                .withTimeout(1)),
                                AlgaeOuttakeCommands.closeArm(algaeOuttake)

                );
        }

        public static Pose2d getClosestReefAlgae(Pose2d currentPose) {
                Pose2d[] branches = /* DriverStation.getAlliance().orElseGet(() -> Alliance.Red) == Alliance.Red */ currentPose
                                .getX() > FieldConstants.fieldLength / 2 ? FieldConstants.Reef.redAlgaePositions
                                                : FieldConstants.Reef.blueAlgaePositions;
                // Get the closest reef to the robot
                double minDistance = Double.MAX_VALUE;
                Pose2d closestReef = FieldConstants.Reef.blueCenterFaces[0];
                for (int i = 0; i < FieldConstants.Reef.blueCenterFaces.length; i++) {
                        double distance = currentPose.getTranslation()
                                        .getDistance(branches[i].getTranslation());
                        if (distance < minDistance) {
                                minDistance = distance;
                                closestReef = branches[i];
                        }
                }

                return closestReef;
        }

        public static Command goToBranch(Drive drive, boolean toLeft) {
                return new DriveCommands.DriveToSuppliedPosition(drive,
                                () -> getClosestReefBranch(drive.getPose(), toLeft))
                                .andThen(DriveCommands.joystickDriveRobotRelative(drive, () -> 0.4, () -> 0, () -> 0)
                                                .withTimeout(0.6));
        }

        public static Pose2d getClosestReefBranch(Pose2d currentPose, boolean toLeft) {
                Pose2d[] branches = /* DriverStation.getAlliance().orElseGet(() -> Alliance.Red) == Alliance.Red */ currentPose
                                .getX() > FieldConstants.fieldLength / 2
                                                ? (toLeft ? FieldConstants.Reef.redLeftBranches
                                                                : FieldConstants.Reef.redRightBranches)
                                                : (toLeft ? FieldConstants.Reef.blueLeftBranches
                                                                : FieldConstants.Reef.blueRightBranches);
                // Get the closest reef to the robot
                double minDistance = Double.MAX_VALUE;
                Pose2d closestReef = FieldConstants.Reef.blueCenterFaces[0];
                for (int i = 0; i < FieldConstants.Reef.blueCenterFaces.length; i++) {
                        double distance = currentPose.getTranslation()
                                        .getDistance(branches[i].getTranslation());
                        if (distance < minDistance) {
                                minDistance = distance;
                                closestReef = branches[i];
                        }
                }
                return closestReef;
        }

}
