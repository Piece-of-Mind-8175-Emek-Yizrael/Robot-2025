package frc.robot.subsystems.Vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionIOReal implements VisionIO 
{
    PhotonCamera camera;
    Transform3d cameraToRobot;
    public PhotonPoseEstimator estimator;

    public VisionIOReal(PhotonCamera camera, Transform3d cameraTransform3d) {
        this.camera = camera;
        this.cameraToRobot = cameraTransform3d;
        estimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape)
            , PoseStrategy.AVERAGE_BEST_TARGETS, cameraTransform3d);
    }

    @Override
    public void updateInputs(VisionInputs inputs) {
        Optional<PhotonPipelineResult> stuff = getMostRecentResult();
        inputs.conected = camera.isConnected();
        if(stuff.isEmpty()) {
            inputs.PhotonResult = null;
        } else {
            inputs.PhotonResult = stuff.get();
        }
    }

    @Override
    public Optional<PhotonPipelineResult> getMostRecentResult() {
        List<PhotonPipelineResult> input = camera.getAllUnreadResults();
        int bestIndex = 0;
        for (int i = 1; i < input.size(); i++) {
            if (input.get(i).getTimestampSeconds() > input.get(bestIndex).getTimestampSeconds()) {
                bestIndex = i;
            }
        }
        return Optional.ofNullable(input.get(bestIndex));
    }    

    @Override
    public Optional<EstimatedRobotPose> getEestimatedPose() {
        Optional<PhotonPipelineResult> stuff = getMostRecentResult();
        if (stuff.isEmpty()) {
            return Optional.ofNullable(null);
        }
        return estimator.update(stuff.get());
    }

    @Override
    public boolean isConnected() {
        return camera.isConnected();
    }
}
