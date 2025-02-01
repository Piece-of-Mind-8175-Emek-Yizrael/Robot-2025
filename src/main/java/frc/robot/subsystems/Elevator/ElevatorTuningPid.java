package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ElevatorTuningPid {

    
    LoggedNetworkNumber kpTune = new LoggedNetworkNumber("kp", 0);
    LoggedNetworkNumber kiTune = new LoggedNetworkNumber("ki", 0);
    LoggedNetworkNumber kdTune = new LoggedNetworkNumber("kd", 0);
    LoggedNetworkNumber kvTune = new LoggedNetworkNumber("kv", 0);
    LoggedNetworkNumber kgTune = new LoggedNetworkNumber("kg", 0);
    LoggedNetworkNumber ksTune = new LoggedNetworkNumber("ks", 0);
    LoggedNetworkNumber maxAccelerationTune = new LoggedNetworkNumber("max acceleration", 0);
    LoggedNetworkNumber maxVelocityTune = new LoggedNetworkNumber("max velocity", 0);


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
    public double getMaxAcceleration(){
        return maxAccelerationTune.get();
    }

    public double getMaxVelocity(){
        return maxVelocityTune.get();
    }

    public void setPidValues(ProfiledPIDController pidController, ElevatorFeedforward feedforward){
        pidController.setP(getKp());
        pidController.setI(getKi());
        pidController.setD(getKd());
        pidController.setConstraints(new TrapezoidProfile.Constraints(getMaxVelocity(), getMaxAcceleration()));
        feedforward = new ElevatorFeedforward(getKs(), getKg(), getKv());
    }

    public void setPidValues(ElevatorFeedforward feedforward){
        feedforward = new ElevatorFeedforward(getKs(), getKg(), getKv());
    }

}
