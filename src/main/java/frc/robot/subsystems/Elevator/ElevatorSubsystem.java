package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {
    public final ElevatorIO elevatorIO;
    public final ElevatorIOInputsAutoLogged elevatorInputs;

    public ElevatorSubsystem(ElevatorIO elevatorIO){
        this.elevatorIO = elevatorIO;
    }

    

}
