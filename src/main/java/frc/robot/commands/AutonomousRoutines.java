package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
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

        public static Command putL2(Drive drive, Elevator elevator, Transfer transfer,
                        boolean proccessorSide) {
                return Commands.sequence(
                                driveToPoseInCorrectAlliance(drive, FieldConstants.Reef.redRightBranches[2],
                                                proccessorSide)
                                                .withTimeout(4)
                                                .alongWith(ElevatorCommands.goToPosition(elevator, 8)),
                                DriveCommands.joystickDriveRobotRelative(drive, () -> 0.35, () -> 0, () -> 0)
                                                .withTimeout(0.3),
                                ElevatorCommands.L2(elevator),
                                TransferCommands.coralOutakeFast(transfer).withTimeout(0.5));
        }

        public static Command putL2ThenIntake(Drive drive, Elevator elevator, Transfer transfer,
                        boolean proccessorSide) {
                Pose2d[] poses = new Pose2d[] { new Pose2d(13.5, 1.5, new Rotation2d(Math.PI)),
                                new Pose2d(16.4, 1.1, Rotation2d.fromDegrees(125)) };
                return Commands.sequence(
                                putL2(drive, elevator, transfer, proccessorSide),
                                Commands.parallel(
                                                ElevatorCommands.closeElevator(elevator),
                                                Commands.sequence(
                                                                driveToPoseInCorrectAlliance(drive, poses[0],
                                                                                proccessorSide)
                                                                                .until(() -> drive.getPose()
                                                                                                .getTranslation()
                                                                                                .getDistance(poses[0]
                                                                                                                .getTranslation()) < 0.5)
                                                                                .withTimeout(2),
                                                                driveToPoseInCorrectAlliance(drive, poses[1],
                                                                                proccessorSide)
                                                                                .withTimeout(3))),
                                TransferCommands.intakeCoral(transfer));
        }

}
