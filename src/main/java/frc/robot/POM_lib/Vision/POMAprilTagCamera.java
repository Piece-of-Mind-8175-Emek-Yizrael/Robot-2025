package frc.robot.POM_lib.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class POMAprilTagCamera extends PhotonCamera {

    PhotonPoseEstimator photonPoseEstimator;

    public POMAprilTagCamera(String name, Transform3d cameraToRobot) throws IOException {
        super(name);
        photonPoseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadFromResource(
                AprilTagFields.k2024Crescendo.m_resourceFile),
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                cameraToRobot);
    }

    public List<POMAprilTag> getListOfVisibleTags() throws IOException {
        List<POMAprilTag> list = new ArrayList<>();
        List<PhotonPipelineResult> pipelineResults = super.getAllUnreadResults();
        PhotonPipelineResult bestResult = getMostRecentResult(pipelineResults);
        for (PhotonTrackedTarget trackedTarget : bestResult.getTargets()) {
            list.add(new POMAprilTag(trackedTarget.getFiducialId(), trackedTarget.getBestCameraToTarget()));
            System.out.println("detected object");
        }
        return list;
    }

    private PhotonPipelineResult getMostRecentResult(List<PhotonPipelineResult> input) {
        int bestIndex = 0;
        for (int i = 1; i < input.size(); i++) {
            if (input.get(i).getTimestampSeconds() > input.get(bestIndex).getTimestampSeconds()) {
                bestIndex = i;
            }
        }
        return input.get(bestIndex);
    }

    public Optional<EstimatedRobotPose> getEstimatedPose() {
        return photonPoseEstimator.update(getMostRecentResult(super.getAllUnreadResults()));
    }

    public POMAprilTag getCertainTag(int desiredID) throws IOException {
        for (POMAprilTag tag : getListOfVisibleTags()) {
            if (tag.id == desiredID) {
                return tag;
            }
        }
        return null;
    }

    public boolean isCertainTagVisible(int desiredID) throws IOException {
        for (POMAprilTag tag : getListOfVisibleTags()) {
            if (tag.id == desiredID) {
                return true;
            }
        }
        return false;
    }
}
