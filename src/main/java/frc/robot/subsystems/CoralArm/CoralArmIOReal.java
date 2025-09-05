package frc.robot.subsystems.CoralArm;

import static frc.robot.subsystems.CoralArm.CoralArmConstants.*;

import java.util.function.BooleanSupplier;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.POM_lib.Motors.POMSparkMax;
import frc.robot.POM_lib.sensors.POMDigitalInput;

public class CoralArmIOReal implements CoralArmIO, Sendable {
    POMSparkMax motor;
    RelativeEncoder encoder;
    private ProfiledPIDController pidController;
    private ArmFeedforward feedforward;
    private POMDigitalInput foldSwitch;
    private BooleanSupplier isCoralIn;

    public CoralArmIOReal(BooleanSupplier isCoralIn) {
        motor = new POMSparkMax(CORAL_ARM_ID);
        feedforward = new ArmFeedforward(KS, KG, KV);
        pidController = new ProfiledPIDController(KP, KI, KD,
                new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));

        encoder = motor.getEncoder();
        this.isCoralIn = isCoralIn;

        foldSwitch = new POMDigitalInput(FOLD_SWITCH);
        pidController.setTolerance(TOLERANCE);// TODO check this

        SparkMaxConfig config = new SparkMaxConfig();

        config.idleMode(IdleMode.kBrake).inverted(INVERTED)
                .smartCurrentLimit(CURRENT_LIMIT)
                .voltageCompensation(VOLTAGE_COMPENSATION);

        // config.softLimit.forwardSoftLimit(FORWARD_SOFT_LIMIT);
        config.encoder.positionConversionFactor(POSITION_CONVERSION_FACTOR)
                .velocityConversionFactor(POSITION_CONVERSION_FACTOR / 60.0);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        resetEncoder();

    }

    @Override
    public void updateInputs(CoralArmIOInputs inputs) {
        inputs.motorConnected = true /* turnConnectedDebouncer.calculate(sparkStickyFault) */;
        inputs.coralArmVelocity = encoder.getVelocity();
        inputs.coralArmPosition = encoder.getPosition();
        inputs.coralArmAppliedVolts = motor.getAppliedOutput() * motor.getBusVoltage(); // FIXME Wont Return Motor
                                                                                        // Voltage
        inputs.foldSwitch = foldSwitch.get();
        resetIfPressed();
    }

    private void resetEncoder() {
        encoder.setPosition(-Math.PI / 2.0);
    }

    @Override
    public void setPercentage(double speed) {
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
        setVoltage(pidController.calculate(encoder.getPosition())
                + getFeedForwardVelocity(pidController.getSetpoint().velocity));
    }

    @Override
    public BooleanSupplier atGoal() {
        return () -> pidController.atGoal();
    }

    @Override
    public void stopMotor() {
        setVoltage(getFeedForwardVelocity(0));// TODO check this
    }

    @Override
    public void resistGravity() {
        setVoltage(getFeedForwardVelocity(0));
    }

    boolean lastSwitchState = false;

    boolean isBrake = true;

    @Override
    public void resetIfPressed() {
        if (foldSwitch.get()) {
            resetEncoder();
        }
    }

    private double getFeedForwardVelocity(double velocity) {
        org.littletonrobotics.junction.Logger.recordOutput("real kG", feedforward.getKg());
        double voltage = feedforward.calculate(encoder.getPosition(), velocity);
        // if (isCoralIn.getAsBoolean()) {
        // voltage += pidConstants.getKgOfCoral();
        // }

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

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Coral Arm");
        builder.addDoubleProperty("P", pidController::getP, pidController::setP);
        builder.addDoubleProperty("I", pidController::getI, pidController::setI);
        builder.addDoubleProperty("D", pidController::getD, pidController::setD);
        builder.addDoubleArrayProperty("Max Velocity, Max Acceleration", () -> {
            Constraints constraints = pidController.getConstraints();
            return new double[] { constraints.maxVelocity, constraints.maxAcceleration };
        },
                (constraints) -> pidController.setConstraints(new Constraints(constraints[0], constraints[1])));

        builder.addDoubleArrayProperty("FeedForward", () -> {
            return new double[] { feedforward.getKg(), feedforward.getKs(), feedforward.getKv() };
        },
                (feedForwardArray) -> {
                    feedforward.setKg(feedForwardArray[0]);
                    feedforward.setKs(feedForwardArray[1]);
                    feedforward.setKv(feedForwardArray[2]);
                });
    }

}