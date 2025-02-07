package frc.robot.POM_lib.Motors;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

public class POMSparkMax extends SparkMax implements POMMotor {
  public POMSparkMax(int id) {
    this(id, MotorType.kBrushless);
  }

  public POMSparkMax(int id, MotorType type) {
    super(id, type);
  }

  @Override
  public void stop() {
    set(0);
  }

  @Override
  public void setDirection(Direction direction) {
    var config = new SparkMaxConfig();
    config.inverted(direction == Direction.CounterClockWise);
    super.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setBrake(boolean isBrake) {
    setBrake(isBrake);
  }
}
