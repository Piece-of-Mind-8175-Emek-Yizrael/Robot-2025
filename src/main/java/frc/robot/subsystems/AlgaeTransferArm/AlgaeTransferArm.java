package frc.robot.subsystems.AlgaeTransferArm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeTransferArm extends SubsystemBase {
    private AlgaeTransferArmIO algaeTransferArmIO;
    private AlgaeTransferArmIOInputsAutoLogged algaeTransferArmInputs = new AlgaeTransferArmIOInputsAutoLogged();

    public AlgaeTransferArm(AlgaeTransferArm algaeTransferArm){
        this.algaeTransferArmIO = algaeTransferArmIO;
    }

    public void periodic() {
        algaeTransferArmIO.updateInputs(algaeTransferArmInputs);
        Logger.processInputs("Algae Transfer", algaeTransferArmInputs);
    }


    public AlgaeTransferArmIO getIO(){
        return algaeTransferArmIO;
    }

    
}
