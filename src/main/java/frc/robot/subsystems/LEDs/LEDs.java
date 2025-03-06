package frc.robot.subsystems.LEDs;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.LEDsCommands;

public class LEDs extends SubsystemBase {

    private final LEDsIO ledsIO;
    private final LEDsIOInputsAutoLogged inputs = new LEDsIOInputsAutoLogged();

    public LEDs(LEDsIO ledsIO, BooleanSupplier leftCamera, BooleanSupplier rightCamera) {
        this.ledsIO = ledsIO;
        setAll(Color.kPurple);
        new Trigger(() -> DriverStation.isEnabled()).onTrue(LEDsCommands.setAll(this, Color.kRed));
        new Trigger(() -> DriverStation.isDisabled()).onTrue(LEDsCommands.setAll(this, Color.kPurple));

        new Trigger(leftCamera).onTrue(LEDsCommands.setFirstHalf(this, Color.kGreen));
        new Trigger(rightCamera).onTrue(LEDsCommands.setSecondHalf(this, Color.kGreen));

        new Trigger(leftCamera).onFalse(LEDsCommands.setFirstHalf(this, Color.kBlack));
        new Trigger(rightCamera).onFalse(LEDsCommands.setSecondHalf(this, Color.kBlack));

    }

    public void setAll(Color color) {
        ledsIO.setAll(color);
    }

    public void setParts(Color... colors) {
        ledsIO.setParts(colors);
    }

    public void blink(Color color, double seconds) {
        ledsIO.blink(color, seconds);
    }

    public void blink(LEDPattern pattern, double seconds) {
        ledsIO.blink(pattern, seconds);
    }

    public void setFirstHalf(Color color) {
        ledsIO.setFirstHalf(color);
    }

    public void setSecondHalf(Color color) {
        ledsIO.setSecondHalf(color);
    }

    @Override
    public void periodic() {
        ledsIO.updateInputs(inputs);
        Logger.processInputs("LEDs", inputs);
    }

}