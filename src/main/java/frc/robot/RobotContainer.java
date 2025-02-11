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

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.POM_lib.Joysticks.PomXboxController;
import frc.robot.commands.AlgaeOuttakeCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElevatorCommands;
import frc.robot.commands.TransferCommands;
import frc.robot.subsystems.AlgaeOuttake.AlgaeOuttake;
import frc.robot.subsystems.AlgaeOuttake.AlgaeOuttakeIO;
import frc.robot.subsystems.AlgaeOuttake.AlgaeOuttakeIOReal;
import frc.robot.subsystems.AlgaeOuttake.AlgaeOuttakeIOSim;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.Elevator.ElevatorIO;
import frc.robot.subsystems.Elevator.ElevatorIOSim;
import frc.robot.subsystems.Elevator.ElevatorReal;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.LEDs.LEDs;
import frc.robot.subsystems.LEDs.LEDsIO;
import frc.robot.subsystems.LEDs.LEDsIOReal;
import frc.robot.subsystems.LEDs.LEDsIOSim;
import frc.robot.subsystems.Transfer.Transfer;
import frc.robot.subsystems.Transfer.TransferIO;
import frc.robot.subsystems.Transfer.TransferIOReal;
import frc.robot.subsystems.Transfer.TransferIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOPOM;
import frc.robot.subsystems.drive.ModuleIOSim;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

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
        private final Drive drive;
        private final AlgaeOuttake algaeOuttake;
        private final Transfer transfer;

        private final LEDs leds;

        // Controller
        private final PomXboxController driverController = new PomXboxController(0);

        // Dashboard inputs
        private final LoggedDashboardChooser<Command> autoChooser;

        private SwerveDriveSimulation driveSimulation;

        private ElevatorSubsystem elevatorSubsystem;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                switch (Constants.currentMode) {
                        case REAL:
                                // Real robot, instantiate hardware IO implementations
                                algaeOuttake = new AlgaeOuttake(new AlgaeOuttakeIOReal());
                                elevatorSubsystem = new ElevatorSubsystem(new ElevatorReal(() -> false));
                                transfer = new Transfer(new TransferIOReal());

                                drive = new Drive(
                                                new GyroIOPigeon(),
                                                new ModuleIOPOM(0),
                                                new ModuleIOPOM(1),
                                                new ModuleIOPOM(2),
                                                new ModuleIOPOM(3));

                                leds = new LEDs(new LEDsIOReal());
                                break;

                        case SIM:
                                // Sim robot, instantiate physics sim IO implementations

                                driveSimulation = new SwerveDriveSimulation(Drive.maplesimConfig,
                                                new Pose2d(3, 3, new Rotation2d()));
                                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
                                // elevatorSubsystem = new ElevatorSubsystem(new ElevatorIOSim());
                                // Logger.recordOutput("Intake Pose", new Pose3d());//FIXME temp

                                elevatorSubsystem = new ElevatorSubsystem(new ElevatorIOSim());
                                transfer = new Transfer(new TransferIOSim(driveSimulation));

                                algaeOuttake = new AlgaeOuttake(new AlgaeOuttakeIOSim());

                                drive = new Drive(
                                                new GyroIOSim(this.driveSimulation.getGyroSimulation()),
                                                new ModuleIOSim(this.driveSimulation.getModules()[0]),
                                                new ModuleIOSim(this.driveSimulation.getModules()[1]),
                                                new ModuleIOSim(this.driveSimulation.getModules()[2]),
                                                new ModuleIOSim(this.driveSimulation.getModules()[3]));

                                leds = new LEDs(new LEDsIOSim());
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
                                elevatorSubsystem = new ElevatorSubsystem(new ElevatorIO() {
                                });

                                transfer = new Transfer(new TransferIO() {
                                });
                                leds = new LEDs(new LEDsIO() {
                                });
                                break;
                }

                // Set up auto routines
                autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser()); // TODO use
                                                                                                            // auto
                                                                                                            // builder

                // Set up SysId routines
                autoChooser.addOption(
                                "Drive Wheel Radius Characterization",
                                DriveCommands.wheelRadiusCharacterization(drive));
                autoChooser.addOption(
                                "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
                autoChooser.addOption(
                                "Drive SysId (Quasistatic Forward)",
                                drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
                autoChooser.addOption(
                                "Drive SysId (Quasistatic Reverse)",
                                drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
                autoChooser.addOption(
                                "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
                autoChooser.addOption(
                                "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
                autoChooser.addOption(
                                "Drive SysId (Quasistatic Steer Forward)",
                                drive.sysIdSteerQuasistatic(SysIdRoutine.Direction.kForward));
                autoChooser.addOption(
                                "Drive SysId (Quasistatic Steer Reverse)",
                                drive.sysIdSteerQuasistatic(SysIdRoutine.Direction.kReverse));
                autoChooser.addOption(
                                "Drive SysId (Dynamic Steer Forward)",
                                drive.sysIdSteerDynamic(SysIdRoutine.Direction.kForward));
                autoChooser.addOption(
                                "Drive SysId (Dynamic Steer Reverse)",
                                drive.sysIdSteerDynamic(SysIdRoutine.Direction.kReverse));

                // Configure the button bindings
                configureButtonBindings();
        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link GenericHID} or one of its subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
         * it to a {@link
         * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
         */
        private void configureButtonBindings() {
                // Default command, normal field-relative drive
                drive.setDefaultCommand(
                                DriveCommands.joystickDrive(
                                                drive,
                                                () -> -driverController.getLeftY() * 0.25,
                                                () -> -driverController.getLeftX() * 0.25,
                                                () -> -driverController.getRightX() * 0.25));

                // driverController.povRight().onTrue(getPathCommand());
                // driverController.povLeft().onTrue(Commands.runOnce(() ->
                // moduleFL.setTurnPosition(new Rotation2d(Math.PI))));
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
                driverController.PovLeft().onTrue(Commands.runOnce(drive::stopWithX, drive));

                // Reset gyro to 0° when Y button is pressed
                driverController.PovUp().onTrue(drive.resetGyroCommand());

                driverController.start().onTrue(AlgaeOuttakeCommands.openArm(algaeOuttake));
                driverController.back().onTrue(AlgaeOuttakeCommands.closeArm(algaeOuttake));
                // driverController.leftTrigger().onTrue(ElevatorCommands.setSpeed(elevatorSubsystem,
                // 0.2));
                // driverController.rightTrigger().onTrue(ElevatorCommands.stopElevator(elevatorSubsystem));
                driverController.y().onTrue(
                                ElevatorCommands.goToPosition(elevatorSubsystem, ElevatorConstants.L3_POSITION));
                driverController.x().onTrue(
                                ElevatorCommands.goToPosition(elevatorSubsystem, ElevatorConstants.L2_POSITION));
                driverController.y().or(driverController.x())
                                .onFalse(ElevatorCommands.closeElevator(elevatorSubsystem));
                driverController.leftTrigger().onTrue(TransferCommands.coralOutake(transfer));
                new Trigger(transfer::isCoralIn).onTrue(TransferCommands.startTransfer(transfer));

                driverController.a()
                                .whileTrue(ElevatorCommands.goToPosition(elevatorSubsystem, 11.5).withTimeout(0.8)
                                                .andThen(DriveCommands.driveBackSlow(drive)
                                                                .raceWith(ElevatorCommands.goToPositionWithoutPid(
                                                                                elevatorSubsystem,
                                                                                ALGAE_OUTTAKE_ELEVATOR_POSITION))));
                // driverController.PovDown().onTrue(ElevatorCommands.goToPosition(elevatorSubsystem,
                // ALGAE_OUTTAKE_ELEVATOR_POSITION));

                // driverController.PovDown().onTrue(ElevatorCommands.goToPositionWithoutPid(elevatorSubsystem,
                // ALGAE_OUTTAKE_ELEVATOR_POSITION));
                driverController.a().onFalse(
                                (ElevatorCommands.closeElevator(elevatorSubsystem)));
                // driverController.PovDown().onFalse(AlgaeOuttakeCommands.closeArm(algaeOuttake)
                // .alongWith(ElevatorCommands.closeElevator(elevatorSubsystem)));

                // leds.setDefaultCommand(LEDsCommands.setAll(leds, Color.kPurple));
        }

        public void displaSimFieldToAdvantageScope() {
                if (Constants.currentMode != Constants.Mode.SIM)
                        return;

                Logger.recordOutput(
                                "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
                Logger.recordOutput(
                                "FieldSimulation/Notes", SimulatedArena.getInstance().getGamePiecesArrayByType("Note"));
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return autoChooser.get();
        }

        public void startTransfer() {
                TransferCommands.startTransfer(transfer).schedule();
        }

        public void closeAlgaeArm() {
                AlgaeOuttakeCommands.closeArm(algaeOuttake).schedule();
        }

        public Command getPathCommand() {
                try {
                        // Load the path you want to follow using its name in the GUI
                        PathPlannerPath path = PathPlannerPath.fromPathFile("Drive 1 meter path");

                        // Create a path following command using AutoBuilder. This will also trigger
                        // event markers.
                        return (AutoBuilder.followPath(path).andThen(Commands.print("After command")))
                                        .beforeStarting(() -> drive.setPose(path.getStartingDifferentialPose()))
                                        .beforeStarting(Commands.print("Starts following path"));
                } catch (Exception e) {
                        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
                        return Commands.print("Exception creating path");
                }
        }

}
