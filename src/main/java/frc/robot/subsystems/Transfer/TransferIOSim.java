package frc.robot.subsystems.Transfer;

import static edu.wpi.first.units.Units.Meters;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;

public class TransferIOSim implements TransferIO {

    // FlywheelSim flywheel;
    IntakeSimulation intakeSimulation;
    SwerveDriveSimulation swerveDriveSimulation;

    public TransferIOSim(SwerveDriveSimulation swerveDriveSimulation) {
        // flywheel = new
        // FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNeo550(1), 0, 50),
        // DCMotor.getNeo550(1), 50.0, 0.1);
        intakeSimulation = IntakeSimulation.InTheFrameIntake("Coral", swerveDriveSimulation, Meters.of(0.7),
                IntakeSide.BACK, 1);
        intakeSimulation.startIntake();
    }

    @Override
    public void setSpeed(double speed) {
        // double flywheelAngularVelocity = speed *
        // flywheel.getGearbox().freeSpeedRadPerSec * flywheel.getGearing();
        // flywheel.setAngularVelocity(flywheelAngularVelocity);
        if (intakeSimulation.obtainGamePieceFromIntake()) {

            ReefscapeCoralOnField coral = new ReefscapeCoralOnField(swerveDriveSimulation.getSimulatedDriveTrainPose());
            SimulatedArena.getInstance().addGamePiece(coral);
        }
    }

    @Override
    public void setVoltage(double voltage) {
        // flywheel.setInputVoltage(voltage);
        if (intakeSimulation.obtainGamePieceFromIntake()) {

            ReefscapeCoralOnField coral = new ReefscapeCoralOnField(swerveDriveSimulation.getSimulatedDriveTrainPose());
            SimulatedArena.getInstance().addGamePiece(coral);
        }
    }

    @Override
    public void stopMotor() {
        // flywheel.setAngularVelocity(0);
    }

    @Override
    public void updateInputs(TransferIOInputs outputs) {
        // outputs.velocity = flywheel.getAngularVelocity().magnitude();
        // outputs.voltage = flywheel.getInputVoltage();
        // outputs.current = flywheel.getCurrentDrawAmps();
        outputs.transferSensorInput = intakeSimulation.getGamePiecesAmount() > 0;

    }

}
