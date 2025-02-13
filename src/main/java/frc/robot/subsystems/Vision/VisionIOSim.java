package frc.robot.subsystems.Vision;

import static frc.robot.subsystems.Vision.VisionConstants.aprilTagLayout;

import java.util.function.Supplier;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionIOSim extends VisionIOReal {

    private static VisionSystemSim visionSim;

  private final Supplier<Pose2d> poseSupplier;
//  private final PhotonCameraSim cameraSim;

    public VisionIOSim(String lCamera, String rCamera, Transform3d rRobotToCamera, Transform3d lRobotToCamera, Supplier<Pose2d> poseSupplier){
        super(lCamera, rCamera, rRobotToCamera, lRobotToCamera);
        this.poseSupplier = poseSupplier;
        if(visionSim == null){
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(aprilTagLayout);

        }
        var cameraProperties = new SimCameraProperties();
//        cameraSim = new PhotonCameraSim(camera,cameraProperties);
//        visionSim.addCamera(cameraSim, robotToCamera);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs){
        visionSim.update(poseSupplier.get());
        super.updateInputs(inputs);
    }

}
