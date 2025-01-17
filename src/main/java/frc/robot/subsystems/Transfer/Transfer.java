package frc.robot.subsystems.Transfer;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Transfer extends SubsystemBase {

    private final TransferIO transferIO;
    private final TransferIOInputsAutoLogged transferInputs = new TransferIOInputsAutoLogged();

    public Transfer(TransferIO transferIO) {
        this.transferIO = transferIO;
    }


    public TransferIO getIO(){
        return transferIO;
    }

    public void setSpeed(double speed) {
        getIO().setSpeed(speed);
    }


    public void setVoltage(double voltage) {
        getIO().setVoltage(voltage);
    }

    public void stopMotor() {
        getIO().stopMotor();
    }

    
    public void periodic() {
        transferIO.updateInputs(transferInputs);
        Logger.processInputs("Transfer", transferInputs);
    }
    
}
