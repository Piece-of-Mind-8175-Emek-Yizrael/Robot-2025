package frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj.simulation.PWMSim;

public class ArmIOSim implements ArmIO {

    private PWMSim sim;

    private final double MAX_ANGLE = 180;
    public ArmIOSim() {
        sim = new PWMSim(0);//TODO Replace with port constant

    }

    @Override
    public void setDegrees(Double Degrees) {
        sim.setPosition(Degrees);

    }

    public double getAngle() {
        return sim.getPosition() * MAX_ANGLE;
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {

        inputs.Degrees = getAngle();
        
    }
    
}
