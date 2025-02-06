package frc.robot.subsystems.Elevator;

import static frc.robot.subsystems.Elevator.ElevatorConstants.*;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ElevatorTuningPid {

    
    LoggedNetworkNumber kpTune = new LoggedNetworkNumber("kp", KP);
    LoggedNetworkNumber kiTune = new LoggedNetworkNumber("ki", KI);
    LoggedNetworkNumber kdTune = new LoggedNetworkNumber("kd", KD);
    LoggedNetworkNumber kvTune = new LoggedNetworkNumber("kv", KV);
    LoggedNetworkNumber kgTune = new LoggedNetworkNumber("kg", KG);
    LoggedNetworkNumber ksTune = new LoggedNetworkNumber("ks", KS);
    LoggedNetworkNumber maxAccelerationTune = new LoggedNetworkNumber("max acceleration", MAX_ACCELERATION);
    LoggedNetworkNumber maxVelocityTune = new LoggedNetworkNumber("max velocity", MAX_VELOCITY);
    LoggedNetworkNumber kgWithCoralTune = new LoggedNetworkNumber("kg with coral", KG_OF_CORAL);
    LoggedNetworkNumber upperKgTune = new LoggedNetworkNumber("upper kg", UPPER_KG);


    public double getKp(){
        return kpTune.get();
    }

    public double getKi(){
        return kiTune.get();
    }

    public double getKd(){
        return kdTune.get();
    }
    public double getKv(){
        return kvTune.get();
    }

    public double getKg(){
        return kgTune.get();
    }

    public double getKgOfCoral(){
        return kgWithCoralTune.get();
    }

    public double getUpperKg(){
        return upperKgTune.get();
    }

    public double getKs(){
        return ksTune.get();
    }
    public double getMaxAcceleration(){
        return maxAccelerationTune.get();
    }

    public double getMaxVelocity(){
        return maxVelocityTune.get();
    }

    
}
