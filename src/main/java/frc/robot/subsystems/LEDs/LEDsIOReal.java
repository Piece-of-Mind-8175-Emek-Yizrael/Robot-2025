package frc.robot.subsystems.LEDs;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;


public class LEDsIOReal implements LEDsIO{
    private final AddressableLED led = new AddressableLED(0); //TODO port
    private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(0); //TODO length
    
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

    @Override
    public void setAll(Color color) {
        setRange(0, ledBuffer.getLength() - 1, color);
    }


    @Override
    public void setParts(Color... colors){
        int ledLength = ledBuffer.getLength() - 1;
        int partLength = ledLength / colors.length;
        int numberOfParts = colors.length;

        for (int i = 1; i <= numberOfParts; i++) {
            setRange((i - 1)*partLength, i*partLength, colors[i]);
        }

    
        

    }

    private void setColor(int idx, Color color) {
        int rgbFactor = 255;
        ledBuffer.setRGB(idx, (int) (color.red*rgbFactor), (int) (color.green*rgbFactor), (int) (color.blue*rgbFactor));
        led.setData(ledBuffer);
    }

    private void setRange(int startIdx, int endIdx, Color color) {
        LEDPattern solidColorPattern = LEDPattern.solid(color);
        
        solidColorPattern.applyTo(ledBuffer);
        led.setData(ledBuffer);
    }
    private void blink(Color color, double seconds) {}
    
    private void blink(Color color1, Color color2, double seconds) {}

    }
