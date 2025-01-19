package frc.robot.subsystems.Arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.simulation.PWMSim;

public class ArmIOSim implements ArmIO {

    private PWMSim sim;

    private final double MAX_ANGLE = 180;

    private Pose3d pose;
    public ArmIOSim() {
        sim = new PWMSim(0);//TODO Replace with port constant
        Logger.recordOutput("Arm/Pose", pose);

    }

    @Override
    public void setDegrees(Double Degrees) {
        sim.setPosition(Degrees);
        pose = new Pose3d(0,0,0, new Rotation3d(0,getAngle(),0));


    }

    public double getAngle() {
        return sim.getPosition() * MAX_ANGLE;
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {

        inputs.Degrees = getAngle();
        
    }
    
}
