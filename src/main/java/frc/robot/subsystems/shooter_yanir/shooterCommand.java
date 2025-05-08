package frc.robot.subsystems.shooter_yanir;

import edu.wpi.first.wpilibj2.command.Command;

public class shooterCommand extends Command {
    private final shooterSubsystem shooterSubsystem;
    private double Speed;

    public shooterCommand(shooterSubsystem shooterSubsystem, double speed) {
        this.shooterSubsystem = shooterSubsystem;
        this.Speed = speed;
    }

    @Override
    public void initialize() {
        // shooterSubsystem.setSpeedtoFlyWheels(15);
        // new java.util.Timer().schedule(new java.util.TimerTask() {
        // @Override
        // public void run() {
        // shooterSubsystem.setSpeedtoFlyWheels(0);
        // }
        // }, 3000);
        shooterSubsystem.setSpeedtoFlyWheels(Speed);
    }

    // int executeCounter = 0;

    @Override
    public void execute() {
        // executeCounter += 1;
    }

    public void end() {
        shooterSubsystem.stopMotor();
    }

    @Override
    public boolean isFinished() {
        // if (executeCounter >= 1) {
        // return true;
        // } else {
        // return false;
        // }
        return false;
    }

}
