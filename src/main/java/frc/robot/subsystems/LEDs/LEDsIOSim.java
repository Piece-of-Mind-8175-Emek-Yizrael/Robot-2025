package frc.robot.subsystems.LEDs;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.subsystems.LEDs.LEDsConstants.LENGTH;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;


public class LEDsIOSim implements LEDsIO{
    private final AddressableLEDBuffer ledBuffer;
    
    public LEDsIOSim() {
        ledBuffer = new AddressableLEDBuffer(LENGTH);

    }

    
    @Override
    public void updateInputs(LEDsIOInputs inputs) {
        inputs.ledColorList = new String[ledBuffer.getLength()];
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            inputs.ledColorList[i] = ledBuffer.getLED(i).toHexString();
        }
    }

    // Sets all LEDs to the same color.
    @Override
    public void setAll(Color color) {
        LEDPattern solidColorPattern = LEDPattern.solid(color);
        
        solidColorPattern.applyTo(ledBuffer);    }


    // Gets a list of Colors and splits them equally across the LED strip
    @Override
    public void setParts(Color... colors){
        int numberOfParts = colors.length;
        Map<Double, Color> map = new HashMap<Double, Color>();
        
        for (int i = 0; i < numberOfParts; i++) {
            map.put(i*(1.0/numberOfParts), colors[i]);
        }

        LEDPattern steps = LEDPattern.steps(map);


        steps.applyTo(ledBuffer);

    }

    // Blinks a color on and off for a given number of seconds
    @Override
    public void blink(Color color, double seconds) {
        LEDPattern pattern = LEDPattern.solid(color);
        blink(pattern, seconds);
    }

    // Blinks a pattern on and off for a given number of seconds
    @Override
    public void blink(LEDPattern pattern, double seconds) {
        LEDPattern blinking = pattern.blink(Seconds.of(seconds));
        blinking.applyTo(ledBuffer);
    }

    // private void setColor(int idx, Color color) {
    //     int rgbFactor = 255;
    //     ledBuffer.setRGB(idx, (int) (color.red*rgbFactor), (int) (color.green*rgbFactor), (int) (color.blue*rgbFactor));
    // }


    }
