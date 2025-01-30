package frc.robot.commands;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Vision.VisionSubsystem;
import frc.robot.subsystems.drive.Drive;

import java.util.ArrayList;
import java.util.Objects;

public class AlignToTargetCommand extends Command {
    public enum POMAlignmentTarget {
        REEF(Constants.REEF_TAG_ID), CORAL(Constants.CORAL_STATION_TAG_ID);

        final int[] tagIds;

        POMAlignmentTarget(int[] tagIds) {
            this.tagIds = tagIds;
        }

        public Pair<Integer, Transform2d>[] getTagsFromVision(VisionSubsystem visionSubsystem, Pose2d robotPosition) {
            ArrayList<Pair<Integer, Pose3d>> tags = visionSubsystem.getAllAprilTags();
            ArrayList<Pair<Integer, Transform2d>> filtered = new ArrayList<>();
            for (Pair<Integer, Pose3d> item : tags) {
                boolean isGood = false;
                for (Integer i : tagIds) {
                    if (Objects.equals(item.getFirst(), i)) {
                        isGood = true;
                        break;
                    }
                }
                if (isGood) {
                    filtered.add(new Pair<>(item.getFirst(), robotPosition.minus(item.getSecond().toPose2d())));
                }
            }
            Pair<Integer, Transform2d>[] returnArr = new Pair[filtered.size()];
            for (int i = 0; i < returnArr.length; i++) {
                returnArr[i] = filtered.get(i);
            }
            return returnArr;
        }
    }

    final Drive driveSubsystem;
    final VisionSubsystem visionSubsystem;
    final POMAlignmentTarget alignmentTarget;
    // still needs to check if this is necessary. currently not implemented
    //    final Transform2d wantedTransform;

    public AlignToTargetCommand(Drive driveSubsystem, VisionSubsystem visionSubsystem, POMAlignmentTarget alignmentTarget /*,  Transform2d wantedTransform */) {
        this.visionSubsystem = visionSubsystem;
        this.driveSubsystem = driveSubsystem;
        this.alignmentTarget = alignmentTarget;
        //        this.wantedTransform = wantedTransform;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        Pair<Integer, Transform2d>[] tags = alignmentTarget.getTagsFromVision(visionSubsystem, driveSubsystem.getPose());
        Pair<Integer, Transform2d> closestTag = tags[0];
        for (int i = 1; i < tags.length; i++) {
            if(closestTag.getSecond().getTranslation().getNorm() > tags[i].getSecond().getTranslation().getNorm()) {
                closestTag = tags[i];
            }
        }
        // the part that spins the robot, requires careful checking.
        DriveCommands.rotateByAngle(driveSubsystem, closestTag.getSecond().getRotation()).execute();
    }

    @Override
    public boolean isFinished() {
        // needs confirmation from uri that this is the correct return value
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        // needs confirmation from uri that this function does not need to be implemented
    }
}
