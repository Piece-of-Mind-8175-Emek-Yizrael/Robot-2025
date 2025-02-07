package frc.robot.subsystems.Elevator;

import static frc.robot.subsystems.Elevator.ElevatorConstants.*;
import java.util.function.BooleanSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import frc.robot.POM_lib.Motors.POMSparkMax;
import frc.robot.POM_lib.sensors.POMDigitalInput;

public class ElevatorRealPid implements ElevatorIO {
    POMSparkMax motor;
    RelativeEncoder encoder = motor.getEncoder();
    private SparkClosedLoopController controller;
    private ElevatorFeedforward feedforward;
    private double currentSetPoint;
    private POMDigitalInput foldSwitch;
    private ElevatorTuningPid pidConstants;
    private BooleanSupplier isCoralIn;

    public ElevatorRealPid(BooleanSupplier isCoralIn) {
        motor = new POMSparkMax(ELEVATOR_ID);
        foldSwitch = new POMDigitalInput(FOLD_SWITCH);

        feedforward = new ElevatorFeedforward(KS, KG, KV);
        // feedforward = new ElevatorFeedforward(pidConstants.getKs(),
        // pidConstants.getKg(), pidConstants.getKv());//TODO
        controller = motor.getClosedLoopController();

        this.isCoralIn = isCoralIn;

        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake).inverted(INVERTED)
                .smartCurrentLimit(CURRENT_LIMIT)
                .voltageCompensation(VOLTAGE_COMPENSATION);

        config.softLimit.forwardSoftLimit(FORWARD_SOFT_LIMIT);
        config.encoder.positionConversionFactor(POSITION_CONVERSION_FACTOR)
                .velocityConversionFactor(POSITION_CONVERSION_FACTOR / 60.0);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder.setPosition(0);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.motorConnected = true /* turnConnectedDebouncer.calculate(sparkStickyFault) */;
        inputs.elevatorVelocity = encoder.getVelocity();
        inputs.elevatorPosition = encoder.getPosition();
        inputs.elevatorAppliedVolts = motor.getAppliedOutput() * motor.getBusVoltage(); // FIXME Wont Return Motor
                                                                                        // Voltage
        inputs.foldSwitch = foldSwitch.get();
        setPidValues();
    }

    @Override
    public void setSpeed(double speed) {
        motor.set(speed);
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void setVoltageWithCoral(double voltage) {
        motor.setVoltage(voltage + KG_OF_CORAL);
    }

    @Override
    public void setGoal(double goal) {
        double ffPower = getFeedForwardVelocity(encoder.getPosition() - goal > 0 ? -1 : 1);
        controller.setReference(goal, ControlType.kPosition, ClosedLoopSlot.kSlot0, ffPower, ArbFFUnits.kVoltage);
        currentSetPoint = goal;
    }

    @Override
    public BooleanSupplier atGoal() {
        return () -> Math.abs(currentSetPoint - encoder.getPosition()) < TOLERANCE;
    }

    @Override
    public void stopMotor() {
        setVoltage(0 + getFeedForwardVelocity(0));// TODO check this
    }

    @Override
    public void resistGravity() {
        setVoltage(getFeedForwardVelocity(0));
    }

    @Override
    public void resetlfPressed() {
        if (foldSwitch.get()) {
            encoder.setPosition(0);
        }
    }

    @Override
    public void setPidValues() {
        feedforward = new ElevatorFeedforward(pidConstants.getKs(), pidConstants.getKg(), pidConstants.getKv());
    }

    private double getFeedForwardVelocity(double velocity) {
        return isCoralIn.getAsBoolean() ? feedforward.calculate(velocity) + KG_OF_CORAL
                : feedforward.calculate(velocity);
    }

    @Override
    public void setFeedForward(double velocity) {
        motor.setVoltage(getFeedForwardVelocity(velocity));
    }

    @Override
    public boolean isPressed() {
        return foldSwitch.get();
    }

    @Override
    public void setVoltageWithResistGravity(double voltage) {
        motor.setVoltage(getFeedForwardVelocity(0) + voltage);
    }

}
