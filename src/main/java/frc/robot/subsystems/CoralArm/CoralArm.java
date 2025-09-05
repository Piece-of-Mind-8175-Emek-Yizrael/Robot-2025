package frc.robot.subsystems.CoralArm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralArm extends SubsystemBase {

    private CoralArmIO io;
    private CoralArmIOInputsAutoLogged inputs;

    public CoralArm() {
        this.io = RobotBase.isReal() ? new CoralArmIOReal(() -> false) : null;
        this.inputs = new CoralArmIOInputsAutoLogged();

        SmartDashboard.putData("Coral Arm", (CoralArmIOReal) io);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Coral Arm", inputs);
        Logger.recordOutput("Current command",
                this.getCurrentCommand() == null ? "null" : this.getCurrentCommand().getName());
    }

    public CoralArmIO getIO() {
        return io;
    }

}
