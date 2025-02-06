package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {

    ElevatorSim elevator;
    ProfiledPIDController pid;
    Pose3d pose;

    public ElevatorIOSim() {
        elevator = new ElevatorSim(DCMotor.getNEO(1), 50, 30, 0.06, 0.6, 1.55, true, 0.6);
        // pid = new ProfiledPIDController(0, 0, 0);
        pose = new Pose3d(0, 0, elevator.getPositionMeters(), new Rotation3d());
    }

    @Override
    public void setSpeed(double speed) {
        elevator.setState(elevator.getPositionMeters(), speed);
    }

    @Override
    public void setVoltage(double voltage) {
        elevator.setInputVoltage(voltage);
    }

    // @Override
    // public void setSetPoint(double setpoint) {
    // // setSpeed(pid.calculate(elevator.getPositionMeters(), setpoint));//FIXME
    // Won't Work, need to find a way around PID probably
    // elevator.setState(0, setpoint);

    // }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        elevator.update(0.02);
        inputs.elevatorPosition = elevator.getPositionMeters();
        inputs.elevatorVelocity = elevator.getVelocityMetersPerSecond();
        inputs.elevatorAppliedVolts = elevator.getOutput(1);// TODO: Calculate Real Simulated Voltage
        inputs.motorConnected = true;
        pose = new Pose3d(0, 0, elevator.getPositionMeters(), new Rotation3d());
        Logger.recordOutput("Elevator/Elevator Pose", pose);// TODO change value of pose realtime
        Logger.recordOutput("Elevator/Elevator position", elevator.getPositionMeters());// TODO change value of pose
                                                                                        // realtime

    }

}
