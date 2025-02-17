package frc.robot.subsystems.Outake;

import static edu.wpi.first.units.Units.Meters;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class OutakeIOSim implements OutakeIO {

    
    FlywheelSim flywheel;
    IntakeSimulation intakeSimulation;
    SwerveDriveSimulation swerveDriveSimulation;

    public OutakeIOSim(SwerveDriveSimulation swerveDriveSimulation) {
        flywheel = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNeo550(1), 0, 50),DCMotor.getNeo550(1), 50.0, 0.1);
        intakeSimulation = IntakeSimulation.InTheFrameIntake("Coral", swerveDriveSimulation, Meters.of(0.7), IntakeSide.BACK, 1);
        intakeSimulation.startIntake();
    }

    @Override
    public void setSpeed(double speed) {
        flywheel.setAngularVelocity(speed);
        if(intakeSimulation.obtainGamePieceFromIntake()){

            // ReefscapeCoral coral = new ReefscapeCoral(swerveDriveSimulation.getSimulatedDriveTrainPose());
            // SimulatedArena.getInstance().addGamePiece(coral);
        }
    }

    @Override
    public void stop() {
        flywheel.setAngularVelocity(0);
    }






    public void updateOutputs(OutakeIOInputs outputs) {
        outputs.outakeVelocityRadPerSec = flywheel.getAngularVelocity().magnitude();
        outputs.outakeAppliedVolts = flywheel.getInputVoltage();
        outputs.outakeCurrentAmps = flywheel.getCurrentDrawAmps();
        outputs.outakeConnected = true;
        
    }

}
