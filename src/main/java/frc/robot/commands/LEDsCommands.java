package frc.robot.commands;

import frc.robot.subsystems.LEDs.LEDs;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;





public class LEDsCommands {

    public static Command setAll(LEDs leds, Color color) {
        return Commands.runOnce(() -> leds.setAll(color), leds);
    }

    public static Command setParts(LEDs leds, Color... colors) {
        return Commands.runOnce(() -> leds.setParts(colors), leds);
    }

    public static Command blink(LEDs leds, Color color, double seconds) {
        return Commands.run(() -> leds.blink(color, seconds), leds);
    }

    public static Command blink(LEDs leds, LEDPattern pattern, double seconds) {
        return Commands.run(() -> leds.blink(pattern, seconds), leds);
    }
    
}
