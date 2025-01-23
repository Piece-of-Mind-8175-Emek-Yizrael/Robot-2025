package frc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.math.geometry.Pose3d;


public interface VisionIO {
    @AutoLog
    public class VisionInputs {
        boolean conected = false;
        PhotonPipelineResult PhotonResult = new PhotonPipelineResult();
    }   

    public default boolean isConnected() {
        return false;
    }

    public default void updateInputs(VisionInputs inputs) {
    }
    
    public default Optional<PhotonPipelineResult> getMostRecentResult() {
        return Optional.ofNullable(null);
    }

    public default Optional<EstimatedRobotPose> getEestimatedPose() {
        return Optional.ofNullable(null);
    }
}
