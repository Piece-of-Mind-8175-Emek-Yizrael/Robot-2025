package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.AlgaeOuttake.AlgaeOuttake;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Transfer.Transfer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.FieldConstants;

public class AutonomousRoutines {
        public static Command driveToPoseInCorrectAlliance(Drive drive, Pose2d pose, boolean proccessorSide) {
                if (proccessorSide) {
                        pose = new Pose2d(pose.getX(), FieldConstants.fieldWidth - pose.getY(),
                                        pose.getRotation().unaryMinus());
                }
                return new ConditionalCommand(new DriveCommands.DriveToPosition(drive, pose),
                                new DriveCommands.DriveToPosition(drive,
                                                new Pose2d(FieldConstants.fieldLength - pose.getX(),
                                                                FieldConstants.fieldWidth - pose.getY(),
                                                                Rotation2d.fromDegrees(180
                                                                                + pose.getRotation().getDegrees()))),
                                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red);
        }

        public static Command driveRobotRelativeCorrectSide(Drive drive, boolean proccessorSide, double x, double y,
                        double omega) {
                return DriveCommands.joystickDrive(drive,
                                () -> x,
                                () -> proccessorSide ? y : -y,
                                () -> proccessorSide ? omega : -omega);
        }

        public static Command putL2(Drive drive, Elevator elevator, Transfer transfer,
                        boolean proccessorSide) {
                return Commands.sequence(
                                driveToPoseInCorrectAlliance(drive, FieldConstants.Reef.redRightBranches[2],
                                                proccessorSide)
                                                .withTimeout(4)
                                                .alongWith(ElevatorCommands.goToPosition(elevator, 8)),
                                DriveCommands.joystickDriveRobotRelative(drive, () -> 0.4, () -> 0, () -> 0)
                                                .withTimeout(0.4),
                                ElevatorCommands.L2(elevator),
                                TransferCommands.coralOutakeFast(transfer).withTimeout(0.5));
        }

        public static Command putL2Twice(Drive drive, Elevator elevator, Transfer transfer,
                        boolean proccessorSide) {
                Pose2d[] poses = new Pose2d[] { new Pose2d(14.8, 1.1, Rotation2d.fromDegrees(125)),
                                new Pose2d(16.7, 1.1, Rotation2d.fromDegrees(125)),
                                new Pose2d(15, 7, Rotation2d.fromDegrees(180)) };
                return Commands.sequence(
                                putL2(drive, elevator, transfer, proccessorSide),
                                Commands.parallel(
                                                ElevatorCommands.closeElevator(elevator),
                                                Commands.sequence(
                                                                driveRobotRelativeCorrectSide(drive, proccessorSide,
                                                                                -0.3, -0.75, -0.4).withTimeout(0.5),
                                                                driveToPoseInCorrectAlliance(drive, poses[1],
                                                                                proccessorSide)
                                                                                .withTimeout(2.1)),
                                                new InstantCommand(drive::stop)),
                                Commands.parallel(
                                                TransferCommands.intakeCoral(transfer),
                                                Commands.sequence(
                                                                new WaitCommand(0.8),
                                                                driveRobotRelativeCorrectSide(drive, proccessorSide,
                                                                                0.3, 0.6, -0.3).withTimeout(1),
                                                                driveToPoseInCorrectAlliance(drive,
                                                                                FieldConstants.Reef.redLeftBranches[0],
                                                                                proccessorSide).withTimeout(2))),
                                DriveCommands.joystickDriveRobotRelative(drive, () -> 0.4, () -> 0, () -> 0)
                                                .withTimeout(0.7),
                                new InstantCommand(() -> drive.runVelocity(new ChassisSpeeds(), true)),
                                ElevatorCommands.L2(elevator),
                                TransferCommands.coralOutakeFast(transfer).withTimeout(0.5));
        }

        public static Command putL2TwiceAlter(Drive drive, Elevator elevator, Transfer transfer,
                        boolean proccessorSide) {
                Pose2d[] poses = new Pose2d[] { new Pose2d(14.8, 1.1, Rotation2d.fromDegrees(125)),
                                new Pose2d(16.3, .8, Rotation2d.fromDegrees(125)),
                                new Pose2d(15, 7, Rotation2d.fromDegrees(180)) };
                return Commands.sequence(
                                putL2(drive, elevator, transfer, proccessorSide),
                                Commands.parallel(
                                                ElevatorCommands.closeElevator(elevator),
                                                Commands.sequence(
                                                                driveRobotRelativeCorrectSide(drive, proccessorSide,
                                                                                -0.3, -0.7, -0.55)
                                                                                .until(() -> poses[0].getTranslation()
                                                                                                .getDistance(drive
                                                                                                                .getPose()
                                                                                                                .getTranslation()) < 1)
                                                                                .withTimeout(1),
                                                                driveToPoseInCorrectAlliance(drive, poses[1],
                                                                                proccessorSide)
                                                                                .withTimeout(2.5))),
                                Commands.parallel(
                                                TransferCommands.intakeCoral(transfer),
                                                Commands.sequence(
                                                                new WaitCommand(1),
                                                                driveRobotRelativeCorrectSide(drive, proccessorSide,
                                                                                0.4, 0.75, -0.3)
                                                                                .until(() -> poses[2].getTranslation()
                                                                                                .getDistance(drive
                                                                                                                .getPose()
                                                                                                                .getTranslation()) < 1)
                                                                                .withTimeout(1.2),
                                                                driveToPoseInCorrectAlliance(drive,
                                                                                FieldConstants.Reef.redLeftBranches[0],
                                                                                proccessorSide).withTimeout(2))),
                                DriveCommands.joystickDriveRobotRelative(drive, () -> 0.4, () -> 0, () -> 0)
                                                .withTimeout(0.4),
                                new InstantCommand(() -> drive.runVelocity(new ChassisSpeeds(), true)),
                                ElevatorCommands.L2(elevator),
                                TransferCommands.coralOutakeFast(transfer).withTimeout(0.5));
        }

        public static Command nearL3PlusAlgae(Drive drive, Elevator elevator, Transfer transfer,
                        AlgaeOuttake algaeOuttake) {
                return Commands.sequence(
                                Commands.parallel(
                                                AlgaeOuttakeCommands.openArm(algaeOuttake),
                                                ElevatorCommands.goToPosition(elevator, 10),
                                                driveToPoseInCorrectAlliance(drive,
                                                                FieldConstants.Reef.redRightBranches[3]
                                                                                .transformBy(new Transform2d(0, 0.1,
                                                                                                new Rotation2d())),
                                                                false)
                                                                .withTimeout(4)),
                                Commands.parallel(
                                                DriveCommands.joystickDriveRobotRelative(drive, () -> 0.4, () -> 0,
                                                                () -> 0)
                                                                .withTimeout(0.4),
                                                ElevatorCommands.goToPosition(elevator, 18).withTimeout(0.8)),
                                Commands.parallel(
                                                DriveCommands.joystickDriveRobotRelative(drive, () -> -0.5, () -> 0,
                                                                () -> 0.3)
                                                                .withTimeout(1),
                                                ElevatorCommands.goToPosition(elevator, 25).withTimeout(1)),
                                AlgaeOuttakeCommands.closeArm(algaeOuttake),
                                new WaitCommand(1),
                                driveToPoseInCorrectAlliance(drive, FieldConstants.Reef.redRightBranches[3],
                                                false)
                                                .withTimeout(4)
                                                .alongWith(ElevatorCommands.goToPosition(elevator, 8)),
                                DriveCommands.joystickDriveRobotRelative(drive, () -> 0.4, () -> 0, () -> 0)
                                                .withTimeout(0.4),
                                ElevatorCommands.L3(elevator).withTimeout(2),
                                TransferCommands.coralOutakeFast(transfer).withTimeout(0.5));
        }

}
