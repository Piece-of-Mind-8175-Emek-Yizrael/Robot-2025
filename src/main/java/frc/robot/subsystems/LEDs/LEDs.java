package frc.robot.subsystems.LEDs;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;


public class LEDs extends SubsystemBase {

    private final LEDsIO ledsIO;
    private final LEDsIOInputsAutoLogged inputs = new LEDsIOInputsAutoLogged();

    public LEDs(LEDsIO ledsIO) {
        this.ledsIO = ledsIO;
    }


    public void setAll(Color color) {
        ledsIO.setAll(color);    
    }



    public void setParts(Color... colors){
        ledsIO.setParts(colors);
    }


    @Override
    public void periodic() {
        ledsIO.updateInputs(inputs);
        Logger.processInputs("LEDs", inputs);
    }

    
}
