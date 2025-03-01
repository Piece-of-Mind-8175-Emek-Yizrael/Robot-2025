package frc.robot.subsystems.Elevator;

import static frc.robot.subsystems.Elevator.ElevatorConstants.*;

import java.util.function.BooleanSupplier;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.POM_lib.Motors.POMSparkMax;
import frc.robot.POM_lib.sensors.POMDigitalInput;

public class ElevatorReal implements ElevatorIO {
    POMSparkMax motor;
    RelativeEncoder encoder;
    private ProfiledPIDController pidController;
    private ElevatorFeedforward feedforward;
    private POMDigitalInput foldSwitch;
    private POMDigitalInput brakeSwitch;
    private ElevatorTuningPid pidConstants;
    private BooleanSupplier isCoralIn;

    public ElevatorReal(BooleanSupplier isCoralIn) {
        motor = new POMSparkMax(ELEVATOR_ID);
        feedforward = new ElevatorFeedforward(KS, KG, KV);
        pidController = new ProfiledPIDController(KP, KI, KD,
                new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));
        // feedforward = new ElevatorFeedforward( pidConstants.getKs(),
        // pidConstants.getKg(), pidConstants.getKv());
        // pidController = new ProfiledPIDController(pidConstants.getKp(),
        // pidConstants.getKi(), pidConstants.getKd(), new
        // TrapezoidProfile.Constraints(pidConstants.getMaxVelocity(),pidConstants.getMaxAcceleration()));

        pidConstants = new ElevatorTuningPid();

        encoder = motor.getEncoder();
        this.isCoralIn = isCoralIn;

        foldSwitch = new POMDigitalInput(FOLD_SWITCH);
        brakeSwitch = new POMDigitalInput(BRAKE_SWITCH);
        pidController.setTolerance(TOLERANCE);// TODO chaeck this

        SparkMaxConfig config = new SparkMaxConfig();

        config.idleMode(IdleMode.kCoast).inverted(INVERTED)
                .smartCurrentLimit(CURRENT_LIMIT)
                .voltageCompensation(VOLTAGE_COMPENSATION);

        // config.softLimit.forwardSoftLimit(FORWARD_SOFT_LIMIT);
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
        resetlfPressed();
    }

    private void resetEncoder() {
        encoder.setPosition(0);
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
        pidController.setGoal(goal);
        setVoltage(getFeedForwardVelocity(pidController.getSetpoint().velocity)
                + pidController.calculate(encoder.getPosition()));
    }

    @Override
    public BooleanSupplier atGoal() {
        return () -> pidController.atGoal();
    }

    @Override
    public void stopMotor() {
        setVoltage(0 + getFeedForwardVelocity(0));// TODO check this
    }

    @Override
    public void resistGravity() {
        setVoltage(getFeedForwardVelocity(0));
    }

    boolean lastSwitchState = false;

    // @Override
    // public void resetlfPressed() {
    // if (DriverStation.isEnabled()) {
    // if (foldSwitch.get()) {
    // encoder.setPosition(0);
    // if (!lastSwitchState) {
    // motor.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake),
    // ResetMode.kNoResetSafeParameters,
    // PersistMode.kNoPersistParameters);
    // }
    // lastSwitchState = true;
    // } else if (lastSwitchState) {
    // motor.configure(new SparkMaxConfig().idleMode(IdleMode.kCoast),
    // ResetMode.kNoResetSafeParameters,
    // PersistMode.kNoPersistParameters);
    // lastSwitchState = false;
    // }
    // } else {
    // motor.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake),
    // ResetMode.kNoResetSafeParameters,
    // PersistMode.kNoPersistParameters);
    // }

    // }

    @Override
    public void resetlfPressed() {
        if (foldSwitch.get()) {
            resetEncoder();
        }
        if (DriverStation.isEnabled()) {
            if (foldSwitch.get()) {
                motor.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake), ResetMode.kNoResetSafeParameters,
                        PersistMode.kNoPersistParameters);
            } else {
                motor.configure(new SparkMaxConfig().idleMode(IdleMode.kCoast), ResetMode.kNoResetSafeParameters,
                        PersistMode.kNoPersistParameters);
            }
        } else {
            if (brakeSwitch.get()) {
                motor.configure(new SparkMaxConfig().idleMode(IdleMode.kCoast),
                        ResetMode.kNoResetSafeParameters,
                        PersistMode.kNoPersistParameters);
                resetEncoder();
            } else {
                motor.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake),
                        ResetMode.kNoResetSafeParameters,
                        PersistMode.kNoPersistParameters);
            }
        }
    }

    @Override
    public void setPidValues() {
        pidController.setP(pidConstants.getKp());
        pidController.setI(pidConstants.getKi());
        pidController.setD(pidConstants.getKd());
        pidController.setConstraints(
                new TrapezoidProfile.Constraints(pidConstants.getMaxVelocity(), pidConstants.getMaxAcceleration()));
        feedforward = new ElevatorFeedforward(pidConstants.getKs(), pidConstants.getKg(), pidConstants.getKv());

    }

    private double getFeedForwardVelocity(double velocity) {
        org.littletonrobotics.junction.Logger.recordOutput("real kG", feedforward.getKg());
        double voltage = feedforward.calculate(velocity);
        if (isCoralIn.getAsBoolean()) {
            voltage += pidConstants.getKgOfCoral();
        }

        if (encoder.getPosition() >= UPPER_POSITION) {
            voltage += pidConstants.getUpperKg();
        }
        return voltage;
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

    public double getPosition() {
        return encoder.getPosition();
    }

    @Override
    public void resetPID() {
        pidController.reset(encoder.getPosition(), encoder.getVelocity());
    }

    @Override
    public void resetPID(double newGoal) {
        if (newGoal - encoder.getPosition() > 0) {
            pidController.reset(encoder.getPosition(), Math.max(encoder.getVelocity(), getFeedForwardVelocity(1)));
        } else {
            pidController.reset(encoder.getPosition(), Math.min(encoder.getVelocity(), getFeedForwardVelocity(1)));
        }
    }

}
