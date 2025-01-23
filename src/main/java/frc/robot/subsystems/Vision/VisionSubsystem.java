package frc.robot.subsystems.Vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    VisionIO frontCamera;
    VisionInputsAutoLogged inptus;
    
    public VisionSubsystem(VisionIO frontCamera) {
        this.frontCamera = frontCamera;
    }

    @Override
    public void periodic() {
        frontCamera.updateInputs(inptus);
    }

    public Optional<Pose3d> getEstimatedPosition() {
        Optional<EstimatedRobotPose> stuff = frontCamera.getEestimatedPose();
        if (stuff.isEmpty()) {
            return Optional.ofNullable(null);
        } else {
            return Optional.of(stuff.get().estimatedPose);
        }
    }
}
