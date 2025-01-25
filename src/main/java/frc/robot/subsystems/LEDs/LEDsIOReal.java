package frc.robot.subsystems.LEDs;

import static frc.robot.subsystems.LEDs.LEDsConstants.LED_PORT;
import static frc.robot.subsystems.LEDs.LEDsConstants.LENGTH;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import static edu.wpi.first.units.Units.*;



public class LEDsIOReal implements LEDsIO{
    private final AddressableLED led = new AddressableLED(LED_PORT);
    private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(LENGTH);
    
    public LEDsIOReal() {
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();

    }

    
    @Override
    public void updateInputs(LEDsIOInputs inputs) {
        inputs.ledColorList = new ArrayList<>(ledBuffer.getLength());
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            inputs.ledColorList.set(i, ledBuffer.getLED(i));
        }
    }

    // Sets all LEDs to the same color.
    @Override
    public void setAll(Color color) {
        setRange(0, ledBuffer.getLength() - 1, color);
    }


    // Gets a list of Colors and splits them equally across the LED strip
    @Override
    public void setParts(Color... colors){
        int ledLength = ledBuffer.getLength() - 1;
        int partLength = ledLength / colors.length;
        int numberOfParts = colors.length;

        for (int i = 1; i <= numberOfParts; i++) {
            setRange((i - 1)*partLength, i*partLength, colors[i]);
        }

    }

    // Blinks a color on and off for a given number of seconds
    @Override
    public void blink(Color color, double seconds) {
        LEDPattern pattern = LEDPattern.solid(color);
        LEDPattern blinking = pattern.blink(Seconds.of(seconds));
        blinking.applyTo(ledBuffer);
        updateLEDs();
    }

    // Blinks a pattern on and off for a given number of seconds
    @Override
    public void blink(LEDPattern pattern, double seconds) {
        LEDPattern blinking = pattern.blink(Seconds.of(seconds));
        blinking.applyTo(ledBuffer);
        updateLEDs();
    }

    private void updateLEDs(){
        led.setData(ledBuffer);
    }

    private void setColor(int idx, Color color) {
        int rgbFactor = 255;
        ledBuffer.setRGB(idx, (int) (color.red*rgbFactor), (int) (color.green*rgbFactor), (int) (color.blue*rgbFactor));
        updateLEDs();
    }

    private void setRange(int startIdx, int endIdx, Color color) {
        LEDPattern solidColorPattern = LEDPattern.solid(color);
        
        solidColorPattern.applyTo(ledBuffer);
        updateLEDs();
    }



    }