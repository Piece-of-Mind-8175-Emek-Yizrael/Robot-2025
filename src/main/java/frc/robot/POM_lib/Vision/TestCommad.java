package frc.robot.POM_lib.Vision;

import java.io.IOException;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;

public class TestCommad extends Command {
    POMAprilTagCamera camera;


    public TestCommad(POMAprilTagCamera camera){
        this.camera = camera;

    }

    @Override
    public void execute(){
        List<POMAprilTag> detectlist = null;
        try {
            detectlist = camera.getListOfVisibleTags();
            for(POMAprilTag tag : detectlist){
                Logger.recordOutput("vision/tag to camera", tag.getId() + " >>  " + tag.getCameraToTag().toString());
                System.out.print(tag.getCameraToTag().toString());  
            }
        } catch (IOException e) {
            Logger.recordOutput("vision/tag to camera", "can not get tags from the camera :(");
        }
        

    }

    @Override
    public boolean isFinished(){
        return false;
    }

}
