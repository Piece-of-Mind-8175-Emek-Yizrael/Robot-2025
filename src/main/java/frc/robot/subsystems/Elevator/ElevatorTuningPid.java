package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ElevatorTuningPid {

    
    LoggedNetworkNumber kpTune = new LoggedNetworkNumber("kp", 0);
    LoggedNetworkNumber kiTune = new LoggedNetworkNumber("ki", 0);
    LoggedNetworkNumber kdTune = new LoggedNetworkNumber("kd", 0);
    LoggedNetworkNumber kvTune = new LoggedNetworkNumber("kv", 0);
    LoggedNetworkNumber kgTune = new LoggedNetworkNumber("kg", 0);
    LoggedNetworkNumber ksTune = new LoggedNetworkNumber("ks", 0);

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

    public double getKs(){
        return ksTune.get();
    }

}
