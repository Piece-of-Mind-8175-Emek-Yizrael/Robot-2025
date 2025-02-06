package frc.robot.subsystems.AlgaeOuttake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeOuttake extends SubsystemBase {
    public final AlgaeOuttakeIO algaeOuttakeIO;
    public final AlgaeOuttakeIOInputsAutoLogged algaeOuttakeInputs = new AlgaeOuttakeIOInputsAutoLogged();

    public AlgaeOuttake(AlgaeOuttakeIO algaeOuttakeIO) {
        this.algaeOuttakeIO = algaeOuttakeIO;
    }

    public double getDegrees(){
        return algaeOuttakeIO.getDegrees();
    }
    
    public void setDegrees(double degrees){
        algaeOuttakeIO.setDegrees(degrees);
    }

    public void periodic() {
        algaeOuttakeIO.updateInputs(algaeOuttakeInputs);
        Logger.processInputs("algae outtake", algaeOuttakeInputs);
    } 
}