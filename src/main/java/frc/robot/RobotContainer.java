// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static frc.robot.subsystems.AlgaeOuttake.AlgaeOuttakeConstants.ALGAE_OUTTAKE_ELEVATOR_POSITION;
import static frc.robot.subsystems.Elevator.ElevatorConstants.L2_POSITION;
import static frc.robot.subsystems.Elevator.ElevatorConstants.MANUAL_FAST_CLOSE;
import static frc.robot.subsystems.Elevator.ElevatorConstants.MANUAL_FAST_OPEN;
import static frc.robot.subsystems.Elevator.ElevatorConstants.MANUAL_SLOW_CLOSE;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.POM_lib.Joysticks.PomXboxController;
import frc.robot.commands.AlgaeOuttakeCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveCommands.DriveToPosition;
import frc.robot.commands.ElevatorCommands;
import frc.robot.commands.TransferCommands;
import frc.robot.subsystems.AlgaeOuttake.AlgaeOuttake;
import frc.robot.subsystems.AlgaeOuttake.AlgaeOuttakeIO;
import frc.robot.subsystems.AlgaeOuttake.AlgaeOuttakeIOReal;
import frc.robot.subsystems.AlgaeOuttake.AlgaeOuttakeIOSim;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.Elevator.ElevatorIO;
import frc.robot.subsystems.Elevator.ElevatorReal;
import frc.robot.subsystems.LEDs.LEDs;
import frc.robot.subsystems.LEDs.LEDsIOSim;
import frc.robot.subsystems.Transfer.Transfer;
import frc.robot.subsystems.Transfer.TransferIO;
import frc.robot.subsystems.Transfer.TransferIOReal;
import frc.robot.subsystems.Transfer.TransferIOSim;
import frc.robot.subsystems.Vision.VisionIOReal;
import frc.robot.subsystems.Vision.VisionIOSim;
import frc.robot.subsystems.Vision.VisionSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOPOM;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.FieldConstants.Reef;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // Subsystems
        private Drive drive;
        private AlgaeOuttake algaeOuttake;
        private Transfer transfer;

        // private final LEDs leds;

        // Controller
        private final PomXboxController driverController = new PomXboxController(0);
        private final PomXboxController operatorController = new PomXboxController(1);

        // Dashboard inputs
        private final LoggedDashboardChooser<Command> autoChooser;

        private SwerveDriveSimulation driveSimulation;

        private Elevator elevatorSubsystem;

        private LEDs leDs;

        private Color color;

        private boolean isRelative;

        private VisionSubsystem vision;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                switch (Constants.currentMode) {
                        case REAL:
                                // Real robot, instantiate hardware IO implementations
                                algaeOuttake = new AlgaeOuttake(new AlgaeOuttakeIOReal());
                                elevatorSubsystem = new Elevator(new ElevatorReal(() -> false));
                                transfer = new Transfer(new TransferIOReal());

                                isRelative = true;
                                drive = new Drive(
                                                new GyroIOPigeon(),
                                                new ModuleIOPOM(0),
                                                new ModuleIOPOM(1),
                                                new ModuleIOPOM(2),
                                                new ModuleIOPOM(3));

                                // leds = new LEDs(new LEDsIOReal());
                                VisionIOReal[] cameras = {
                                                // new VisionIOReal("Left Front Camera",
                                                // Constants.VisionConstants.l_camera_transform),
                                                new VisionIOReal("Right Front Camera",
                                                                Constants.VisionConstants.r_camera_transform), };
                                vision = new VisionSubsystem(drive::addVisionMeasurement, cameras);

                        case SIM:
                                // Sim robot, instantiate physics sim IO implementations

                                // driveSimulation = new SwerveDriveSimulation(Drive.maplesimConfig,
                                // new Pose2d(3, 3, new Rotation2d()));
                                // SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

                                // drive = new Drive(
                                // new GyroIOSim(this.driveSimulation.getGyroSimulation()),
                                // new ModuleIOSim(this.driveSimulation.getModules()[0]),
                                // new ModuleIOSim(this.driveSimulation.getModules()[1]),
                                // new ModuleIOSim(this.driveSimulation.getModules()[2]),
                                // new ModuleIOSim(this.driveSimulation.getModules()[3]));

                                // vision = new VisionSubsystem(drive::addVisionMeasurement,
                                // new VisionIOSim("camera_0",
                                // new Transform3d(0.2, 0.0, 0.2,
                                // new Rotation3d(0.0, 0.0, Math.PI)),
                                // driveSimulation::getSimulatedDriveTrainPose));
                                // transfer = new Transfer(new TransferIOSim(driveSimulation));
                                // algaeOuttake = new AlgaeOuttake(new AlgaeOuttakeIOSim());

                                // leds = new LEDs(new LEDsIOSim());

                                break;

                        default:
                                // Replayed robot, disable IO implementations
                                algaeOuttake = new AlgaeOuttake(new AlgaeOuttakeIO() {
                                });

                                drive = new Drive(
                                                new GyroIO() {
                                                },
                                                new ModuleIO() {
                                                },
                                                new ModuleIO() {
                                                },
                                                new ModuleIO() {
                                                },
                                                new ModuleIO() {
                                                });
                                elevatorSubsystem = new Elevator(new ElevatorIO() {
                                });

                                transfer = new Transfer(new TransferIO() {
                                });
                                // leds = new LEDs(new LEDsIO() {
                                // });
                                break;
                }

                // Set up auto routines
                autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser()); // TODO use
                                                                                                            // auto
                                                                                                            // builder

                // Set up SysId routines
                // autoChooser.addOption(
                // "Drive Wheel Radius Characterization",
                // DriveCommands.wheelRadiusCharacterization(drive));
                // autoChooser.addOption(
                // "Drive Simple FF Characterization",
                // DriveCommands.feedforwardCharacterization(drive));
                // autoChooser.addOption(
                // "Drive SysId (Quasistatic Forward)",
                // drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
                // autoChooser.addOption(
                // "Drive SysId (Quasistatic Reverse)",
                // drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
                // autoChooser.addOption(
                // "Drive SysId (Dynamic Forward)",
                // drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
                // autoChooser.addOption(
                // "Drive SysId (Dynamic Reverse)",
                // drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
                // autoChooser.addOption(
                // "Drive SysId (Quasistatic Steer Forward)",
                // drive.sysIdSteerQuasistatic(SysIdRoutine.Direction.kForward));
                // autoChooser.addOption(
                // "Drive SysId (Quasistatic Steer Reverse)",
                // drive.sysIdSteerQuasistatic(SysIdRoutine.Direction.kReverse));
                // autoChooser.addOption(
                // "Drive SysId (Dynamic Steer Forward)",
                // drive.sysIdSteerDynamic(SysIdRoutine.Direction.kForward));
                // autoChooser.addOption(
                // "Drive SysId (Dynamic Steer Reverse)",
                // drive.sysIdSteerDynamic(SysIdRoutine.Direction.kReverse));

                autoChooser.addOption("drive out",
                                DriveCommands.joystickDriveRobotRelative(drive, () -> 0.22, () -> 0, () -> 0)
                                                .withTimeout(1.2));
                autoChooser.addOption("put L2", new ConditionalCommand(
                                new DriveToPosition(drive, Reef.blueLeftBranches[3]),
                                new DriveToPosition(drive, Reef.redLeftBranches[3]),
                                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue)
                                .andThen(ElevatorCommands.goToPosition(elevatorSubsystem, L2_POSITION))
                                .andThen(TransferCommands.coralOutakeFast(transfer)));
                // Configure the button bindings
                configureButtonBindings();
        }

        private void configureButtonBindings() {

                // driver controller buttens

                // Default command, normal field-relative drive
                drive.setDefaultCommand(
                                DriveCommands.joystickDrive(
                                                drive,
                                                () -> driverController.getLeftY() * 0.35,
                                                () -> driverController.getLeftX() * 0.35,
                                                () -> driverController.getRightX() * 0.32));

                driverController.leftTrigger().whileTrue(
                                DriveCommands.joystickDrive(
                                                drive,
                                                () -> driverController.getLeftY() * 0.25,
                                                () -> driverController.getLeftX() * 0.25,
                                                () -> driverController.getRightX() * 0.25));

                driverController.rightTrigger().whileTrue(
                                DriveCommands.joystickDrive(
                                                drive,
                                                () -> driverController.getLeftY() * 0.6,
                                                () -> driverController.getLeftX() * 0.6,
                                                () -> driverController.getRightX() * 0.35));

                // driverController.povRight().onTrue(getPathCommand());
                driverController.x().whileTrue(DriveCommands.locateToReefCommand(drive, true));
                driverController.b().whileTrue(DriveCommands.locateToReefCommand(drive, false));

                driverController.x().or(driverController.b()).onFalse(new InstantCommand(() -> {
                }, drive));

                // driverController.povLeft().onTrue(Commands.runOnce(() ->
                // moduleFL.setTurnPosition(new Rotation2d(Math.PI)).l;p.));
                // driverController.povRight().onTrue(
                // Commands.runOnce(() -> moduleFL.setTurnPosition(new Rotation2d(1.5 *
                // Math.PI))));
                // driverController.povUp().whileTrue(Commands.run(() ->
                // moduleFL.setTurnPosition(
                // new Rotation2d(driverController.getLeftX(), driverController.getLeftY()))));

                // drive.setDefaultCommand(drive.testSteeringCommand(driverController::getLeftX,
                // driverController::getLeftY));
                // driverController.PovUp().whileTrue(drive.testSteeringCommand(() -> 0, () ->
                // 1));
                // driverController.PovLeft().whileTrue(drive.testSteeringCommand(() -> 1, () ->
                // 0));

                // drive.setDefaultCommand(drive.testSteeringAngleCommand(Rotation2d.fromDegrees(30)));

                // ])));

                // Lock to 0° when A button is held
                // driverController
                // .a()
                // .whileTrue(
                // DriveCommands.joystickDriveAtAngle(
                // drive,
                // () -> -driverController.getLeftY(),
                // () -> -driverController.getLeftX(),
                // () -> new Rotation2d()));

                // Switch to X pattern when X button is pressed
                // driverController.PovLeft().onTrue(Commands.runOnce(drive::stopWithX, drive));

                // Reset gyro to 0° when Y button is pressed
                driverController.PovUp().onTrue(drive.resetGyroCommand(Rotation2d.fromDegrees(180)));
                driverController.PovDown().onTrue(drive.resetGyroCommand(new Rotation2d()));
                driverController.PovLeft().onTrue(drive.resetGyroCommand(Rotation2d.fromDegrees(125)));
                driverController.PovRight().onTrue(drive.resetGyroCommand(Rotation2d.fromDegrees(-125)));

                driverController.LB().whileTrue(
                                DriveCommands.joystickDriveRobotRelative(drive, () -> 0, () -> 0.24, () -> 0));
                driverController.RB().whileTrue(
                                DriveCommands.joystickDriveRobotRelative(drive, () -> 0, () -> -0.24, () -> 0));

                driverController.a().debounce(1)
                                .onTrue(new InstantCommand(() -> drive.disableGoodVision()).ignoringDisable(true));

                // driverController.RB().whileTrue(DriveCommands.joystickDriveRobotRelative(
                // drive,
                // () -> -driverController.getLeftY() * 0.3,
                // () -> -driverController.getLeftX() * 0.3,
                // () -> -driverController.getRightX() * 0.25));

                // operator controller buttens

                // algae arm open & close
                operatorController.LB().onTrue(AlgaeOuttakeCommands.openArm(algaeOuttake));
                operatorController.RB().onTrue(AlgaeOuttakeCommands.closeArm(algaeOuttake));

                // outake algae
                operatorController.a()
                                .whileTrue(ElevatorCommands.goToPosition(elevatorSubsystem, 11.5).withTimeout(0.8)
                                                .andThen(DriveCommands.driveBackSlow(drive)
                                                                .raceWith(ElevatorCommands.goToPositionWithoutPid(
                                                                                elevatorSubsystem,
                                                                                ALGAE_OUTTAKE_ELEVATOR_POSITION))));

                // L2, L3, close with pid
                operatorController.y().onTrue(
                                ElevatorCommands.goToPosition(elevatorSubsystem, ElevatorConstants.L3_POSITION));
                operatorController.x().onTrue(ElevatorCommands.closeElevator(elevatorSubsystem));
                operatorController.b().onTrue(
                                ElevatorCommands.goToPosition(elevatorSubsystem, ElevatorConstants.L2_POSITION));

                // intake coral
                operatorController.PovRight().whileTrue(TransferCommands.coralOutake(transfer));

                // rutern the coral back
                operatorController.PovLeft().whileTrue(TransferCommands.takeCoralIn(transfer));

                // manual elevator control
                // fast
                operatorController.leftTrigger()
                                .whileTrue(ElevatorCommands.closeElevatorManual(elevatorSubsystem, MANUAL_FAST_CLOSE));

                operatorController.rightTrigger()
                                .whileTrue(ElevatorCommands.openElevatorManual(elevatorSubsystem, MANUAL_FAST_OPEN));

                // slow FIXME not working
                // operatorController.PovUp().whileTrue(ElevatorCommands.openElevatorManual(elevatorSubsystem,
                // MANUAL_SLOW_OPEN));

                operatorController.PovUp().whileTrue(TransferCommands.coralOutakeFast(transfer));

                operatorController.PovDown()
                                .whileTrue(ElevatorCommands.closeElevatorManual(elevatorSubsystem, MANUAL_SLOW_CLOSE));

                // driverController.a().onTrue(LEDsCommands.setAll(leDs,color));

                // driverController.leftTrigger().onTrue(ElevatorCommands.setSpeed(elevatorSubsystem,
                // 0.2));
                // driverController.rightTrigger().onTrue(ElevatorCommands.stopElevator(elevatorSubsystem));

                // driverController.y().or(driverController.x())
                // .onFalse(ElevatorCommands.closeElevator(elevatorSubsystem));
                // driverController.leftTrigger().whileTrue(TransferCommands.coralOutake(transfer));
                // new
                // Trigger(transfer::isCoralIn).onTrue(TransferCommands.startTransfer(transfer));

                // driverController.PovDown().onTrue(ElevatorCommands.goToPosition(elevatorSubsystem,
                // ALGAE_OUTTAKE_ELEVATOR_POSITION));

                // driverController.PovDown().onTrue(ElevatorCommands.goToPositionWithoutPid(elevatorSubsystem,
                // ALGAE_OUTTAKE_ELEVATOR_POSITION));
                // operatorController.a().onFalse(
                // (ElevatorCommands.closeElevator(elevatorSubsystem)));
                // driverController.PovDown().onFalse(AlgaeOuttakeCommands.closeArm(algaeOuttake)
                // .alongWith(ElevatorCommands.closeElevator(elevatorSubsystem)));

                // leds.setDefaultCommand(LEDsCommands.setAll(leds, Color.kPurple));

        }

        public void displaSimFieldToAdvantageScope() {
                if (Constants.currentMode != Constants.Mode.SIM)
                        return;

                // Logger.recordOutput(
                // "FieldSimulation/RobotPosition",
                // driveSimulation.getSimulatedDriveTrainPose());
                // Logger.recordOutput(
                // "FieldSimulation/Notes",
                // SimulatedArena.getInstance().getGamePiecesArrayByType("Note"));
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return autoChooser.get();
        }

        // public void startTransfer() {
        // TransferCommands.startTransfer(transfer).schedule();
        // }

        public void closeAlgaeArm() {
                AlgaeOuttakeCommands.closeArm(algaeOuttake).schedule();
        }

        public Command getPathCommand() {
                try {
                        // Load the path you want to follow using its name in the GUI
                        // PathPlannerPath path = PathPlannerPath.fromPathFile("Drive 1 meter path");
                        PathPlannerPath path = PathPlannerPath.fromPathFile("Test1");

                        // Create a path following command using AutoBuilder. This will also trigger
                        // event markers.
                        return (AutoBuilder.followPath(path).andThen(Commands.print("After command")))
                                        .beforeStarting(() -> drive.setPose(path.getStartingDifferentialPose()))
                                        .beforeStarting(Commands.print("Starts following path"));

                } catch (Exception e) {
                        // DriverStataion.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
                        return Commands.print("Exception creating path");
                }
        }

}
