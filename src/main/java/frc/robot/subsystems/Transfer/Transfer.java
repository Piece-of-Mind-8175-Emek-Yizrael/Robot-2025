package frc.robot.subsystems.Transfer;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.subsystems.Transfer.TransferConstants.CORAL_INTAKE_SPEED;
import static frc.robot.subsystems.Transfer.TransferConstants.CORAL_OUTTAKE_SPEED;


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
