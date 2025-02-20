package frc.robot.POM_lib.Vision;

import edu.wpi.first.math.geometry.Transform3d;

public class POMAprilTag {

    Transform3d cameraToTag;
    int id;

    public POMAprilTag(int id, Transform3d cameraToTag) {
        this.id = id;
        this.cameraToTag = cameraToTag;
    }

    public Transform3d getCameraToTag() {
        return cameraToTag;
    }

    public void setCameraToTag(Transform3d cameraToTag) {
        this.cameraToTag = cameraToTag;
    }

    public int getId() {
        return id;
    }

    public void setId(int id) {
        this.id = id;
    }

    @Override
    public String toString() {
        return id + ": (" + cameraToTag.toString() + ")";
    }
}
