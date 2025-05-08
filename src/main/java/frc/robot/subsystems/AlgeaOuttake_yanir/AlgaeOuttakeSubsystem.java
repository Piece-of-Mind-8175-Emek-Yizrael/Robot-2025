
package frc.robot.subsystems.AlgeaOuttake_yanir;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeOuttakeSubsystem extends SubsystemBase {

    Servo servo;

    public AlgaeOuttakeSubsystem() {
        servo = new Servo(0);
    }

    public void setDegrees(double degrees) {
        servo.setAngle(degrees);
    }

    public double getDegrees() {
        return servo.getAngle();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ServoDegrees", getDegrees());
    }
}