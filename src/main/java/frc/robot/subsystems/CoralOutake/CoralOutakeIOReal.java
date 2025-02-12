// import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;

import frc.robot.POM_lib.Motors.POMSparkMax;
import frc.robot.POM_lib.sensors.POMDigitalInput;

public class CoralOutakeIOReal implements CoralOutakeIO{
     private POMDigitalInput isCoralIn = new POMDigitalInput(0); // TODO
     private POMSparkMax coralOutake = new POMSparkMax(0);// TODO

    @Override
    public void updateInputs(CoralOutakeIOInputs inputs){
        inputs.isCoralIn = isCoralIn.get();
    }

    @Override
    public boolean isCoralIn(){
       return isCoralIn.get();
    }

   
    @Override
    public void setPower(double power){
        coralOutake.set(power);
    }
    
}