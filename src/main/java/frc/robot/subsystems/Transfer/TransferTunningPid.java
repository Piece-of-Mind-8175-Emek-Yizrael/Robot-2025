package frc.robot.subsystems.Transfer;

import static frc.robot.subsystems.Transfer.TransferConstants.*;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;


public class TransferTunningPid {
    LoggedNetworkNumber kpTune = new LoggedNetworkNumber("kp", KP);
    LoggedNetworkNumber kiTune = new LoggedNetworkNumber("ki", KI);
    LoggedNetworkNumber kdTune = new LoggedNetworkNumber("kd", KD);
    LoggedNetworkNumber maxAccelerationTune = new LoggedNetworkNumber("max acceleration", MAX_ACCELERATION);
    LoggedNetworkNumber maxVelocityTune = new LoggedNetworkNumber("max velocity", MAX_VELOCITY);


    public double getKp(){
        return kpTune.get();
    }

    public double getKi(){
        return kiTune.get();
    }

    public double getKd(){
        return kdTune.get();
    }

    public double getMaxAcceleration(){
        return maxAccelerationTune.get();
    }

    public double getMaxVelocity(){
        return maxVelocityTune.get();
    }
}
