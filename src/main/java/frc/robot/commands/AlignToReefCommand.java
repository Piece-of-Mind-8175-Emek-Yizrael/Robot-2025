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

public class AlignToReefCommand extends Command {
    public enum POMAlignmentTarget {
        REEF(Constants.REEF_TAG_ID), CORAL(Constants.CORAL_STATION_TAG_ID);

        int[] tagIds;

        POMAlignmentTarget(int[] tagIds) {
            this.tagIds = tagIds;
        }

        public Pair<Integer, Transform2d>[] getTagsFromVision(VisionSubsystem visionSubsystem, Pose2d robotPosition) {
            ArrayList<Pair<Integer, Pose3d>> tags = visionSubsystem.getAllAprilTags();
            ArrayList<Pair<Integer, Transform2d>> filtered = new ArrayList<>();
            for (Pair<Integer, Pose3d> item : tags) {
                boolean isGood = false;
                for (Integer i : tagIds) {
                    if (item.getFirst() == i) {
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
    final Transform2d wantedTransform;

    public AlignToReefCommand(Drive driveSubsystem, VisionSubsystem visionSubsystem, POMAlignmentTarget alignmentTarget, Transform2d wantedTransform) {
        this.visionSubsystem = visionSubsystem;
        this.driveSubsystem = driveSubsystem;
        this.alignmentTarget = alignmentTarget;
        this.wantedTransform = wantedTransform;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        Pair<Integer, Transform2d>[] tags = alignmentTarget.getTagsFromVision(visionSubsystem, driveSubsystem.getPose());
        Pair<Integer, Transform2d> closestTag = tags[0];
        for (int i = 1; i < tags.length; i++) {
            if(closestTag.getSecond().getTranslation().getNorm() > tags[i].getSecond().getTranslation().getNorm()) {
                closestTag = tags[i];
            }
        }
        DriveCommands.rotateByAngle().execute();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
