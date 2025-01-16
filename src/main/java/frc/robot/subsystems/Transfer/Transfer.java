package frc.robot.subsystems.Transfer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
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

    public Command startTransfer() {
        return startEnd(() -> setSpeed(0.1), () ->stopMotor()).until(() -> getIO().isCoralIn());
    }

    public Command coralOutake() {
        return startEnd(() -> setSpeed(0.3), () -> stopMotor()).until(() -> !getIO().isCoralIn());
    }

    public void periodic() {
        transferIO.updateInputs(transferInputs);
        Logger.processInputs("Transfer", transferInputs);
    }
    
}
