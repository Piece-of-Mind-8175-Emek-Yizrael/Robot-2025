package frc.robot.subsystems.AlgaeTransfer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.AlgaeTransfer.AlgaeTransferIO.AlgaeTransferIOInputs;

public class AlgaeTransfer extends SubsystemBase {
    private AlgaeTransferIO algaeTransferIO;
    private AlgaeTransferIOInputs algaeTransferInputs = new AlgaeTransferIOInputs();

    public AlgaeTransfer(AlgaeTransfer algaeTransfer){
        this.algaeTransferIO = algaeTransferIO;
    }

    public void periodic() {
        algaeTransferIO.updateInputs(algaeTransferInputs);
        Logger.processInputs("Algae Transfer", algaeTransferInputs);
    }


    public AlgaeTransferIO getIO(){
        return algaeTransferIO;
    }
}
