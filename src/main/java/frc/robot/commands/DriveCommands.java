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

package frc.robot.commands;

import static frc.robot.subsystems.drive.DriveConstants.ALGAE_OUTTAKE_DRIVE_BACK_SPEED;
import static frc.robot.subsystems.drive.DriveConstants.KD_OMEGA;
import static frc.robot.subsystems.drive.DriveConstants.KD_XY;
import static frc.robot.subsystems.drive.DriveConstants.KI_OMEGA;
import static frc.robot.subsystems.drive.DriveConstants.KI_XY;
import static frc.robot.subsystems.drive.DriveConstants.KP_OMEGA;
import static frc.robot.subsystems.drive.DriveConstants.KP_XY;
import static frc.robot.subsystems.drive.DriveConstants.MAX_ACCELERATION_OMEGA;
import static frc.robot.subsystems.drive.DriveConstants.MAX_ACCELERATION_XY;
import static frc.robot.subsystems.drive.DriveConstants.MAX_VELOCETY_OMEGA;
import static frc.robot.subsystems.drive.DriveConstants.MAX_VELOCETY_XY;
import static frc.robot.subsystems.drive.DriveConstants.OMEGA_TOLERANCE;
import static frc.robot.subsystems.drive.DriveConstants.TRANSLATION_TOLERANCE;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double ANGLE_KP = 5.0;
  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  private DriveCommands() {
  }

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  /**
   * Rotates the robot to a desired angle using PID control.
   *
   * @param drive       The drive subsystem.
   * @param targetAngle The target angle.
   * @return The command that will rotate the robot to the target angle.
   */
  public static Command rotateToAngle(Drive drive, Supplier<Rotation2d> targetAngle) {
    return joystickDriveAtAngle(drive, () -> 0, () -> 0, targetAngle);
  }

  public static Command rotateByAngle(Drive drive, Rotation2d byAngle) {
    ProfiledPIDController angleController = new ProfiledPIDController(
        ANGLE_KP,
        0.0,
        ANGLE_KD,
        new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);
    return new Command() {
      Rotation2d target = new Rotation2d();

      @Override
      public void initialize() {
        target = drive.getRotation().plus(byAngle);
        angleController.reset(drive.getRotation().getRadians());
      }

      @Override
      public void execute() {
        // Calculate angular speed
        double omega = angleController.calculate(drive.getRotation().getRadians(), target.getRadians());

        // Convert to field relative speeds & send command
        ChassisSpeeds speeds = new ChassisSpeeds(0, 0, omega);
        drive.runVelocity(speeds, true);
      }

      @Override
      public void end(boolean interrupted) {
        drive.runVelocity(new ChassisSpeeds(0, 0, 0), true);
      }

      @Override
      public boolean isFinished() {
        return angleController.atSetpoint();
      }
    };
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and
   * angular velocities).
   */
  public static Command joystickDrive(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity = getLinearVelocityFromJoysticks(xSupplier.getAsDouble(),
              ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds = new ChassisSpeeds(
              linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
              linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
              omega * drive.getMaxAngularSpeedRadPerSec());
          boolean isFlipped = DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == Alliance.Red;
          speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
              speeds,
              isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation());
          drive.runVelocity(speeds, true);
        },
        drive).beforeStarting(Commands.runOnce(drive::resetKinematics, drive));
  }

  public static Command joystickDriveRobotRelative(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity = getLinearVelocityFromJoysticks(xSupplier.getAsDouble(),
              ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds = new ChassisSpeeds(
              linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
              linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
              omega * drive.getMaxAngularSpeedRadPerSec());
          boolean isFlipped = DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(speeds, true);
        },
        drive).beforeStarting(Commands.runOnce(drive::resetKinematics, drive));
  }

  public static Command joystickDriveClosedLoopVel(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity = getLinearVelocityFromJoysticks(xSupplier.getAsDouble(),
              ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds = new ChassisSpeeds(
              linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
              linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
              omega * drive.getMaxAngularSpeedRadPerSec());
          boolean isFlipped = DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == Alliance.Red;
          speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
              speeds,
              isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation());
          drive.runVelocity(speeds, false);
        },
        drive).beforeStarting(Commands.runOnce(drive::resetKinematics, drive));
  }

  /**
   * Field relative drive command using joystick for linear control and PID for
   * angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target,
   * or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController = new ProfiledPIDController(
        ANGLE_KP,
        0.0,
        ANGLE_KD,
        new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity = getLinearVelocityFromJoysticks(xSupplier.getAsDouble(),
              ySupplier.getAsDouble());

          // Calculate angular speed
          double omega = angleController.calculate(
              drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds = new ChassisSpeeds(
              linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
              linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
              omega);
          boolean isFlipped = DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, isFlipped
              ? drive.getRotation().plus(new Rotation2d(Math.PI))
              : drive.getRotation()), true);
        },
        drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  public static Command driveBackSlow(Drive drive) {
    ChassisSpeeds speeds = new ChassisSpeeds(ALGAE_OUTTAKE_DRIVE_BACK_SPEED, 0, 0);
    return Commands.runEnd(() -> drive.runVelocity(speeds, true), () -> drive.stop(), drive);
  }

  /**
   * Field relative drive command using joystick for linear control and PID for
   * angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target,
   * or controlling
   * absolute rotation with a joystick. pass 4 booleans to set the robot angle to
   * the front, back,
   * left, or right
   */
  public static Command joystickDriveFixedAngles(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Boolean> front,
      Supplier<Boolean> back,
      Supplier<Boolean> left,
      Supplier<Boolean> right) {

    // Create PID controller
    ProfiledPIDController angleController = new ProfiledPIDController(
        ANGLE_KP,
        0.0,
        ANGLE_KD,
        new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    Rotation2d[] rot = new Rotation2d[] { new Rotation2d(0) }; // Array for accessing in lambda
    // Construct command
    return Commands.run(
        () -> { // TODO: verify angles, maybe need to rotate half pi
          if (front.get())
            rot[0] = new Rotation2d(0.0);
          else if (back.get())
            rot[0] = new Rotation2d(Math.PI);
          else if (left.get())
            rot[0] = new Rotation2d(Math.PI / 2.0);
          else if (right.get())
            rot[0] = new Rotation2d(-Math.PI / 2.0);
          // Get linear velocity
          Translation2d linearVelocity = getLinearVelocityFromJoysticks(xSupplier.getAsDouble(),
              ySupplier.getAsDouble());

          // Calculate angular speed
          double omega = angleController.calculate(drive.getRotation().getRadians(), rot[0].getRadians());

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds = new ChassisSpeeds(
              linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
              linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
              omega);
          boolean isFlipped = DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, isFlipped
              ? drive.getRotation().plus(new Rotation2d(Math.PI))
              : drive.getRotation()), true);
        },
        drive)
        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>
   * This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
            () -> {
              drive.runCharacterization(0.0);
            },
            drive)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
            () -> {
              double voltage = timer.get() * FF_RAMP_RATE;
              drive.runCharacterization(voltage);
              velocitySamples.add(drive.getFFCharacterizationVelocity());
              voltageSamples.add(voltage);
            },
            drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed), true);
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                () -> {
                  var rotation = drive.getRotation();
                  state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                  state.lastAngle = rotation;
                })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius = (state.gyroDelta * DriveConstants.driveBaseRadius) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }

  public static Command goToPosition(Drive drive, Pose3d positions) {

    ChassisSpeeds speeds = new ChassisSpeeds(
        new ProfiledPIDController(KP_XY, KI_XY, KD_XY,
            new TrapezoidProfile.Constraints(MAX_VELOCETY_XY, MAX_ACCELERATION_XY)).calculate(positions.getX()),
        new ProfiledPIDController(KP_XY, KI_XY, KD_XY,
            new TrapezoidProfile.Constraints(MAX_VELOCETY_XY, MAX_ACCELERATION_XY)).calculate(positions.getY()),
        new ProfiledPIDController(KP_OMEGA, KI_OMEGA, KD_OMEGA,
            new TrapezoidProfile.Constraints(MAX_VELOCETY_OMEGA, MAX_ACCELERATION_OMEGA))
            .calculate(positions.getRotation().getAngle()));
    return Commands.runEnd(() -> drive.runVelocity(speeds, true), () -> drive.stop(), drive);
  }

  public class DriveToPosition extends Command {
    private final Drive m_drive;
    private final Pose2d m_target;
    private final ProfiledPIDController m_controllerX;
    private final ProfiledPIDController m_controllerY;
    private final ProfiledPIDController m_controllerTheta;
    private final Timer m_timer = new Timer();

    LoggedNetworkNumber kpTune = new LoggedNetworkNumber("tunes/translation kp", KP_XY);
    LoggedNetworkNumber kiTune = new LoggedNetworkNumber("tunes/translation ki", KI_XY);
    LoggedNetworkNumber kdTune = new LoggedNetworkNumber("tunes/translation kd", KD_XY);
    LoggedNetworkNumber maxVelocityTune = new LoggedNetworkNumber("tunes/translation max velocity", MAX_VELOCETY_XY);
    LoggedNetworkNumber maxAccelerationTune = new LoggedNetworkNumber("tunes/translation max acceleration",
        MAX_ACCELERATION_XY);

    LoggedNetworkNumber kpThetaTune = new LoggedNetworkNumber("tunes/rotation kp", KP_OMEGA);
    LoggedNetworkNumber kiThetaTune = new LoggedNetworkNumber("tunes/rotation ki", KI_OMEGA);
    LoggedNetworkNumber kdThetaTune = new LoggedNetworkNumber("tunes/rotation kd", KD_OMEGA);
    LoggedNetworkNumber maxVelocityThetaTune = new LoggedNetworkNumber("tunes/rotation max velocity",
        MAX_VELOCETY_OMEGA);
    LoggedNetworkNumber maxAccelerationThetaTune = new LoggedNetworkNumber("tunes/rotation max acceleration",
        MAX_ACCELERATION_OMEGA);

    LoggedNetworkNumber translationTolerance = new LoggedNetworkNumber("tunes/translation tolerance",
        TRANSLATION_TOLERANCE);
    LoggedNetworkNumber omegaTolerance = new LoggedNetworkNumber("tunes/rotation tolerance", OMEGA_TOLERANCE);

    public DriveToPosition(Drive drive, Pose2d target) {
      m_drive = drive;
      m_target = target;
      m_controllerX = new ProfiledPIDController(KP_XY, KI_XY, KD_XY,
          new TrapezoidProfile.Constraints(MAX_VELOCETY_XY, MAX_ACCELERATION_XY));
      m_controllerX.setTolerance(TRANSLATION_TOLERANCE);
      m_controllerY = new ProfiledPIDController(KP_XY, KI_XY, KD_XY,
          new TrapezoidProfile.Constraints(MAX_VELOCETY_XY, MAX_ACCELERATION_XY));
      m_controllerY.setTolerance(TRANSLATION_TOLERANCE);
      m_controllerTheta = new ProfiledPIDController(KP_OMEGA, KI_OMEGA, KD_OMEGA,
          new TrapezoidProfile.Constraints(MAX_VELOCETY_OMEGA, MAX_ACCELERATION_OMEGA));
      m_controllerTheta.setTolerance(OMEGA_TOLERANCE);
      m_controllerTheta.enableContinuousInput(-Math.PI, Math.PI);
      addRequirements(drive);
    }

    @Override
    public void initialize() {
      m_timer.reset();
      m_timer.start();
    }

    @Override
    public void execute() {
      m_controllerX.setPID(kpTune.get(), kiTune.get(), kdTune.get());
      m_controllerX.setConstraints(new TrapezoidProfile.Constraints(maxVelocityTune.get(), maxAccelerationTune.get()));
      m_controllerY.setPID(kpTune.get(), kiTune.get(), kdTune.get());
      m_controllerY.setConstraints(new TrapezoidProfile.Constraints(maxVelocityTune.get(), maxAccelerationTune.get()));
      m_controllerTheta.setPID(kpThetaTune.get(), kiThetaTune.get(), kdThetaTune.get());
      m_controllerTheta.setConstraints(
          new TrapezoidProfile.Constraints(maxVelocityThetaTune.get(), maxAccelerationThetaTune.get()));

      var pose = m_drive.getPose();
      var chassisSpeeds = new ChassisSpeeds(
          m_controllerX.calculate(pose.getTranslation().getX(), m_target.getTranslation().getX()),
          m_controllerY.calculate(pose.getTranslation().getY(), m_target.getTranslation().getY()),
          m_controllerTheta.calculate(pose.getRotation().getRadians(), m_target.getRotation().getRadians()));
      m_drive.runVelocity(chassisSpeeds, true);
    }

    @Override
    public void end(boolean interrupted) {
      m_drive.stop();
    }

    @Override
    public boolean isFinished() {
      // return m_timer.hasElapsed(5) ||
      return (m_controllerX.atGoal() && m_controllerY.atGoal() && m_controllerTheta.atGoal());
    }
  }
}
