package frc.robot.subsystems.AlgaeOuttake;
import static frc.robot.subsystems.AlgaeOuttake.AlgaeOuttakeConstants.OUTTAKE_SERVO_CHANNEL;

import edu.wpi.first.wpilibj.Servo;

public class AlgaeOuttakeIOReal implements AlgaeOuttakeIO {
    private final Servo outtakeServo = new Servo(OUTTAKE_SERVO_CHANNEL);

    public AlgaeOuttakeIOReal(){
        outtakeServo.setAngle(OUTTAKE_SERVO_CHANNEL);
    }

    @Override
    public double getDegrees() {
        return outtakeServo.getAngle();
    }


    @Override
    public void setDegrees(double degrees){
        outtakeServo.setAngle(degrees);
    }

    @Override
    public void updateInputs(AlgaeOuttakeIOInputs inputs){
        inputs.degrees = outtakeServo.getAngle();
    }
}
